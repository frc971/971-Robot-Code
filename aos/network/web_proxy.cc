#include "aos/network/web_proxy.h"

#include "glog/logging.h"

#include "aos/flatbuffer_merge.h"
#include "aos/network/connect_generated.h"
#include "aos/network/web_proxy_generated.h"
#include "aos/network/web_proxy_utils.h"
#include "aos/seasocks/seasocks_logger.h"
#include "internal/Embedded.h"

extern "C" {
#include <rawrtc.h>

#define DEBUG_LEVEL 7
#define DEBUG_MODULE "web-proxy"
#include <re_dbg.h>
struct list *tmrl_get(void);
}

DEFINE_int32(proxy_port, 1180, "Port to use for the web proxy server.");
DEFINE_int32(pre_send_messages, 10000,
             "Number of messages / queue to send to a client before waiting on "
             "confirmation that the initial message was received. If set to "
             "-1, will not throttle messages at all. This prevents a situation "
             "where, when run on localhost, the large number of WebRTC packets "
             "can overwhelm the browser and crash the webpage.");

namespace aos {
namespace web_proxy {
WebsocketHandler::WebsocketHandler(::seasocks::Server *server,
                                   aos::EventLoop *event_loop,
                                   StoreHistory store_history,
                                   int per_channel_buffer_size_bytes)
    : server_(server),
      config_(aos::CopyFlatBuffer(event_loop->configuration())),
      event_loop_(event_loop) {
  if (VLOG_IS_ON(2)) {
    dbg_init(DBG_DEBUG, DBG_ALL);
  }
  CHECK_RAWRTC(rawrtc_init(true));

  // We need to reference findEmbeddedContent() to make the linker happy...
  findEmbeddedContent("");
  const aos::Node *self = event_loop_->node();

  subscribers_.reserve(event_loop_->configuration()->channels()->size());
  for (uint i = 0; i < event_loop_->configuration()->channels()->size(); ++i) {
    auto channel = event_loop_->configuration()->channels()->Get(i);
    if (aos::configuration::ChannelIsReadableOnNode(channel, self)) {
      auto fetcher = event_loop_->MakeRawFetcher(channel);
      subscribers_.emplace_back(std::make_unique<aos::web_proxy::Subscriber>(
          std::move(fetcher), i, store_history,
          per_channel_buffer_size_bytes < 0
              ? -1
              : per_channel_buffer_size_bytes / channel->max_size()));
    } else {
      subscribers_.emplace_back(nullptr);
    }
  }
  TimerHandler *const timer = event_loop_->AddTimer([this]() {
    for (auto &subscriber : subscribers_) {
      if (subscriber) subscriber->RunIteration(recording_);
    }
  });

  event_loop_->OnRun([this, timer]() {
    timer->Schedule(event_loop_->monotonic_now(),
                    std::chrono::milliseconds(100));
  });
}

void WebsocketHandler::onConnect(::seasocks::WebSocket *sock) {
  std::unique_ptr<ApplicationConnection> connection =
      std::make_unique<ApplicationConnection>(server_, sock, subscribers_,
                                              config_, event_loop_);

  connections_.insert({sock, std::move(connection)});
}

void WebsocketHandler::onData(::seasocks::WebSocket *sock, const uint8_t *data,
                              size_t size) {
  const FlatbufferSpan<WebSocketMessage> message({data, size});
  if (!message.Verify()) {
    LOG(ERROR) << "Invalid WebsocketMessage received from browser.";
    return;
  }
  VLOG(1) << "Got msg " << aos::FlatbufferToJson(message);
  switch (message.message().payload_type()) {
    case Payload::WebSocketSdp: {
      const WebSocketSdp *offer = message.message().payload_as_WebSocketSdp();
      if (offer->type() != SdpType::OFFER) {
        LOG(WARNING) << "Got the wrong sdp type from client";
        break;
      }
      const flatbuffers::String *sdp = offer->payload();
      connections_[sock]->OnSdp(sdp->c_str());
      break;
    }
    case Payload::WebSocketIce: {
      const WebSocketIce *ice = message.message().payload_as_WebSocketIce();
      connections_[sock]->OnIce(ice);
      break;
    }
    default: {
      break;
    }
  }
}

void WebsocketHandler::onDisconnect(::seasocks::WebSocket *sock) {
  connections_.erase(sock);
}

// Global epoll pointer
static aos::internal::EPoll *global_epoll = nullptr;

static int ReFdListen(int fd, int flags, fd_h *fh, void *arg) {
  if (flags & 0x1) {
    global_epoll->OnReadable(fd, [fh, arg]() { (*fh)(0x1, arg); });
  }
  if (flags & 0x2) {
    global_epoll->OnWriteable(fd, [fh, arg]() { (*fh)(0x2, arg); });
  }
  if (flags & 0x4) {
    global_epoll->OnError(fd, [fh, arg]() { (*fh)(0x4, arg); });
  }
  return 0;
}

static void ReFdClose(int fd) {
  CHECK(global_epoll != nullptr);
  global_epoll->DeleteFd(fd);
}

WebProxy::WebProxy(aos::EventLoop *event_loop, StoreHistory store_history,
                   int per_channel_buffer_size_bytes)
    : WebProxy(event_loop, &internal_epoll_, store_history,
               per_channel_buffer_size_bytes) {}

WebProxy::WebProxy(aos::ShmEventLoop *event_loop, StoreHistory store_history,
                   int per_channel_buffer_size_bytes)
    : WebProxy(event_loop, event_loop->epoll(), store_history,
               per_channel_buffer_size_bytes) {}

WebProxy::WebProxy(aos::EventLoop *event_loop, aos::internal::EPoll *epoll,
                   StoreHistory store_history,
                   int per_channel_buffer_size_bytes)
    : epoll_(epoll),
      server_(std::make_shared<aos::seasocks::SeasocksLogger>(
          ::seasocks::Logger::Level::Info)),
      websocket_handler_(new WebsocketHandler(
          &server_, event_loop, store_history, per_channel_buffer_size_bytes)) {
  CHECK(!global_epoll);
  global_epoll = epoll;

  re_fd_set_listen_callback(&ReFdListen);
  re_fd_set_close_callback(&ReFdClose);

  epoll->BeforeWait([]() {
    const uint64_t to = tmr_next_timeout(tmrl_get());
    if (to != 0) {
      VLOG(3) << "Next timeout " << to;
    }
    // Note: this only works because we are spinning on it...
    // TODO(austin): If we choose to actually sleep, use a timerfd reserved just
    // for handling tmr.
    tmr_poll(tmrl_get());
  });

  server_.addWebSocketHandler("/ws", websocket_handler_);
  CHECK(server_.startListening(FLAGS_proxy_port));

  epoll->OnReadable(server_.fd(), [this]() {
    CHECK(::seasocks::Server::PollResult::Continue == server_.poll(0));
  });

  if (&internal_epoll_ == epoll) {
    TimerHandler *const timer = event_loop->AddTimer([this]() {
      // Run the epoll poller until there are no more events (if we are being
      // backed by a shm event loop, there won't be anything registered to
      // internal_epoll_ and this will just return false).
      // We just deal with clearing all the epoll events using a simulated
      // timer. This does mean that we will spin rather than actually sleeping
      // in any coherent manner, which will be particularly noticeable when past
      // the end of processing other events.
      while (internal_epoll_.Poll(false)) {
        continue;
      }
    });

    event_loop->OnRun([timer, event_loop]() {
      timer->Schedule(event_loop->monotonic_now(),
                      std::chrono::milliseconds(10));
    });
  }
}

WebProxy::~WebProxy() {
  epoll_->DeleteFd(server_.fd());
  server_.terminate();
  CHECK(::seasocks::Server::PollResult::Terminated == server_.poll(0));
  CHECK(global_epoll == epoll_);
  global_epoll = nullptr;
}

void WebProxy::StopRecording() { websocket_handler_->StopRecording(); }

void Subscriber::RunIteration(bool fetch_new) {
  if (fetch_new) {
    if (channels_.empty() && (buffer_size_ == 0 || !store_history_)) {
      fetcher_->Fetch();
      message_buffer_.clear();
      return;
    }

    while (fetcher_->FetchNext()) {
      // If we aren't building up a buffer, short-circuit the FetchNext().
      if (buffer_size_ == 0) {
        fetcher_->Fetch();
      }
      Message message;
      message.index = fetcher_->context().queue_index;
      VLOG(2) << "Packing a message with "
              << GetPacketCount(fetcher_->context()) << "packets";
      for (int packet_index = 0;
           packet_index < GetPacketCount(fetcher_->context()); ++packet_index) {
        // Pack directly into the mbuffer.  This is admittedly a bit painful.
        const size_t packet_size =
            PackedMessageSize(fetcher_->context(), packet_index);
        struct mbuf *mbuffer = mbuf_alloc(packet_size);

        {
          // Wrap a pre-allocated builder around the mbuffer.
          PreallocatedAllocator allocator(mbuf_buf(mbuffer), packet_size);
          flatbuffers::FlatBufferBuilder fbb(packet_size, &allocator);
          flatbuffers::Offset<MessageHeader> message_offset = PackMessage(
              &fbb, fetcher_->context(), channel_index_, packet_index);
          fbb.Finish(message_offset);

          // Now, the flatbuffer is built from the back to the front.  So any
          // extra memory will be at the front.  Set up the end and start
          // pointers on the mbuf.
          mbuf_set_end(mbuffer, packet_size);
          mbuf_set_pos(mbuffer, packet_size - fbb.GetSize());
        }

        message.data.emplace_back(
            std::shared_ptr<struct mbuf>(mbuffer, mem_deref));
      }
      message_buffer_.push_back(std::move(message));
      // If we aren't keeping a buffer, then we should only do one iteration of
      // the while loop--otherwise, if additional messages arrive between the
      // first FetchNext() and the second iteration then we can end up behaving
      // poorly (since we do a Fetch() when buffer_size_ == 0).
      if (buffer_size_ == 0) {
        break;
      }
    }
  }
  for (auto &conn : channels_) {
    std::shared_ptr<ScopedDataChannel> rtc_channel = conn.first.lock();
    CHECK(rtc_channel) << "data_channel was destroyed too early.";
    ChannelInformation *channel_data = &conn.second;
    if (channel_data->transfer_method == TransferMethod::SUBSAMPLE) {
      SkipToLastMessage(channel_data);
    }
    std::shared_ptr<struct mbuf> buffer = NextBuffer(channel_data);
    while (buffer) {
      // TODO(austin): This is a nop so we just buffer forever.  Fix this when
      // we care.
      if (rtc_channel->buffered_amount() > 14000000) {
        VLOG(1) << "skipping a send because buffered amount is too high";
        break;
      }

      rtc_channel->Send(buffer.get());
      buffer = NextBuffer(channel_data);
    }
  }
  if (buffer_size_ >= 0) {
    while (message_buffer_.size() > static_cast<size_t>(buffer_size_)) {
      message_buffer_.pop_front();
    }
  }
}

void Subscriber::AddListener(std::shared_ptr<ScopedDataChannel> data_channel,
                             TransferMethod transfer_method) {
  ChannelInformation info;
  info.transfer_method = transfer_method;

  channels_.emplace_back(std::make_pair(data_channel, info));

  data_channel->set_on_message(
      [this, index = channels_.size() - 1](
          struct mbuf *const buffer,
          const enum rawrtc_data_channel_message_flag /*flags*/) {
        FlatbufferSpan<ChannelState> message(
            {mbuf_buf(buffer), mbuf_get_left(buffer)});
        if (!message.Verify()) {
          LOG(ERROR) << "Invalid flatbuffer received from browser client.";
          return;
        }

        channels_[index].second.reported_queue_index =
            message.message().queue_index();
        channels_[index].second.reported_packet_index =
            message.message().packet_index();
      });
}

void Subscriber::RemoveListener(
    std::shared_ptr<ScopedDataChannel> data_channel) {
  channels_.erase(
      std::remove_if(
          channels_.begin(), channels_.end(),
          [data_channel](const std::pair<std::weak_ptr<ScopedDataChannel>,
                                         ChannelInformation> &channel) {
            return channel.first.lock().get() == data_channel.get();
          }),
      channels_.end());
}

std::shared_ptr<struct mbuf> Subscriber::NextBuffer(
    ChannelInformation *channel) {
  CHECK_NOTNULL(channel);
  if (message_buffer_.empty()) {
    return nullptr;
  }
  const uint32_t earliest_index = message_buffer_.front().index;
  const uint32_t latest_index = message_buffer_.back().index;
  const bool fell_behind = channel->current_queue_index < earliest_index;
  if (fell_behind) {
    channel->current_queue_index = earliest_index;
    channel->next_packet_number = 0;
    return message_buffer_.front().data.at(0);
  }
  // TODO(james): Handle queue index wrapping when >2^32 messages are sent on a
  // channel.
  if (channel->current_queue_index > latest_index) {
    // We are still waiting on the next message to appear; return.
    return nullptr;
  }
  if (FLAGS_pre_send_messages > 0) {
    // Don't buffer up an excessive number of messages to the client.
    // This currently ignores the packet index (and really, any concept of
    // message size), but the main goal is just to avoid locking up the client
    // browser, not to be ultra precise about anything. It's also not clear that
    // message *size* is necessarily even the determining factor in causing
    // issues.
    if (channel->reported_queue_index + FLAGS_pre_send_messages <
        channel->current_queue_index) {
      return nullptr;
    }
  }
  CHECK_EQ(latest_index - earliest_index + 1, message_buffer_.size())
      << "Inconsistent queue indices.";
  const size_t packets_in_message =
      message_buffer_[channel->current_queue_index - earliest_index]
          .data.size();
  CHECK_LT(0u, packets_in_message);
  CHECK_LT(channel->next_packet_number, packets_in_message);

  std::shared_ptr<struct mbuf> original_data =
      message_buffer_[channel->current_queue_index - earliest_index].data.at(
          channel->next_packet_number);

  ++channel->next_packet_number;
  if (channel->next_packet_number == packets_in_message) {
    ++channel->current_queue_index;
    channel->next_packet_number = 0;
  }

  // Trigger a copy of the mbuf without copying the data.
  return std::shared_ptr<struct mbuf>(mbuf_alloc_ref(original_data.get()),
                                      mem_deref);
}

void Subscriber::SkipToLastMessage(ChannelInformation *channel) {
  CHECK_NOTNULL(channel);
  if (message_buffer_.empty() ||
      channel->current_queue_index == message_buffer_.back().index) {
    return;
  }
  channel->current_queue_index = message_buffer_.back().index;
  channel->reported_queue_index = message_buffer_.back().index;
  channel->next_packet_number = 0;
}

ApplicationConnection::ApplicationConnection(
    ::seasocks::Server *server, ::seasocks::WebSocket *sock,
    const std::vector<std::unique_ptr<Subscriber>> &subscribers,
    const aos::FlatbufferDetachedBuffer<aos::Configuration> &config,
    const EventLoop *event_loop)
    : server_(server),
      sock_(sock),
      subscribers_(subscribers),
      config_headers_(PackBuffer(config.span())),
      event_loop_(event_loop) {
  connection_.set_on_negotiation_needed([]() {
    VLOG(1) << "Negotiation needed, not offering so not creating offer.";
  });

  connection_.set_on_local_candidate(
      [this](struct rawrtc_peer_connection_ice_candidate *const candidate,
             char const *const url) { LocalCandidate(candidate, url); });

  connection_.set_on_data_channel(
      [this](std::shared_ptr<ScopedDataChannel> channel) {
        OnDataChannel(channel);
      });

  connection_.Open();
}

ApplicationConnection::~ApplicationConnection() {
  for (auto &it : channels_) {
    it.second.data_channel->Close();
    it.second.data_channel = nullptr;
  }

  // Eh, we are done, tell the channel to shut down.  If we didn't, it would
  // just hang around until the connection closes, which is rather shortly
  // after.
  if (channel_) {
    channel_->Close();
  }
}

void ApplicationConnection::OnSdp(const char *sdp) {
  struct rawrtc_peer_connection_description *remote_description = NULL;

  auto error = rawrtc_peer_connection_description_create(
      &remote_description, RAWRTC_SDP_TYPE_OFFER, sdp);
  if (error) {
    LOG(WARNING) << "Cannot parse remote description: "
                 << rawrtc_code_to_str(error);
    return;
  }

  CHECK_RAWRTC(rawrtc_peer_connection_set_remote_description(
      connection_.connection(), remote_description));

  struct rawrtc_peer_connection_description *local_description;
  CHECK_RAWRTC(rawrtc_peer_connection_create_answer(&local_description,
                                                    connection_.connection()));
  CHECK_RAWRTC(rawrtc_peer_connection_set_local_description(
      connection_.connection(), local_description));

  enum rawrtc_sdp_type type;
  char *local_sdp = nullptr;
  // Get SDP type & the SDP itself
  CHECK_RAWRTC(rawrtc_peer_connection_description_get_sdp_type(
      &type, local_description));
  CHECK_RAWRTC(rawrtc_peer_connection_description_get_sdp(&local_sdp,
                                                          local_description));

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<WebSocketSdp> sdp_fb =
      CreateWebSocketSdpDirect(fbb, SdpType::ANSWER, local_sdp);
  flatbuffers::Offset<WebSocketMessage> answer_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketSdp, sdp_fb.Union());

  VLOG(1) << aos::FlatbufferToJson(
      flatbuffers::GetTemporaryPointer(fbb, answer_message));
  fbb.Finish(answer_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
  mem_deref(local_sdp);
}

void ApplicationConnection::OnIce(const WebSocketIce *ice) {
  if (!ice->has_candidate()) {
    return;
  }
  uint8_t sdp_m_line_index = ice->sdp_m_line_index();

  struct rawrtc_peer_connection_ice_candidate *ice_candidate = nullptr;
  CHECK_RAWRTC(rawrtc_peer_connection_ice_candidate_create(
      &ice_candidate, ice->candidate()->c_str(), ice->sdp_mid()->c_str(),
      &sdp_m_line_index, nullptr));

  rawrtc_peer_connection_add_ice_candidate(connection_.connection(),
                                           ice_candidate);

  mem_deref(ice_candidate);
}

void ApplicationConnection::LocalCandidate(
    struct rawrtc_peer_connection_ice_candidate *const candidate,
    char const *const url) {
  struct rawrtc_ice_candidate *ortc_candidate = nullptr;
  if (candidate) {
    CHECK_RAWRTC(rawrtc_peer_connection_ice_candidate_get_ortc_candidate(
        &ortc_candidate, candidate));

    flatbuffers::FlatBufferBuilder fbb;
    char *sdpp = nullptr;
    CHECK_RAWRTC(
        rawrtc_peer_connection_ice_candidate_get_sdp(&sdpp, candidate));
    char *midp = nullptr;
    CHECK_RAWRTC(
        rawrtc_peer_connection_ice_candidate_get_sdp_mid(&midp, candidate));

    uint8_t media_line_index;
    enum rawrtc_code error =
        rawrtc_peer_connection_ice_candidate_get_sdp_media_line_index(
            &media_line_index, candidate);

    flatbuffers::Offset<flatbuffers::String> sdpp_offset =
        fbb.CreateString(sdpp);
    flatbuffers::Offset<flatbuffers::String> sdp_mid_offset =
        fbb.CreateString(midp);

    WebSocketIce::Builder web_socket_ice_builder(fbb);

    web_socket_ice_builder.add_candidate(sdpp_offset);
    web_socket_ice_builder.add_sdp_mid(sdp_mid_offset);

    if (error == RAWRTC_CODE_SUCCESS) {
      web_socket_ice_builder.add_sdp_m_line_index(media_line_index);
    }
    flatbuffers::Offset<WebSocketIce> ice_offset =
        web_socket_ice_builder.Finish();

    flatbuffers::Offset<WebSocketMessage> ice_message =
        CreateWebSocketMessage(fbb, Payload::WebSocketIce, ice_offset.Union());
    VLOG(1) << url << ": "
            << aos::FlatbufferToJson(
                   flatbuffers::GetTemporaryPointer(fbb, ice_message));
    fbb.Finish(ice_message);

    server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));

    mem_deref(sdpp);
    mem_deref(midp);
  }
}

void ApplicationConnection::OnDataChannel(
    std::shared_ptr<ScopedDataChannel> channel) {
  if (channel->label() == std::string_view("signalling")) {
    CHECK(!channel_);
    channel_ = channel;

    channel_->set_on_message(
        [this](struct mbuf *const buffer,
               const enum rawrtc_data_channel_message_flag flags) {
          HandleSignallingData(buffer, flags);
        });

    channel_->set_on_open([this]() {
      for (const auto &header : config_headers_) {
        channel_->Send(header.buffer());
      }
    });

    channel_->set_on_error([this]() { LOG(ERROR) << "Error on " << this; });

    // Register an on_close callback which does nothing but keeps channel alive
    // until it is done.  This keeps the memory around until rawrtc can finish
    // calling the close callback.
    channel_->set_on_close([channel]() {});
  } else {
    channel_->set_on_close([channel]() {});
    channel->Close();
  }
}

void ApplicationConnection::HandleSignallingData(
    struct mbuf *const
        buffer,  // nullable (in case partial delivery has been requested)
    const enum rawrtc_data_channel_message_flag /*flags*/) {
  FlatbufferSpan<SubscriberRequest> message(
      {mbuf_buf(buffer), mbuf_get_left(buffer)});
  if (!message.Verify()) {
    LOG(ERROR) << "Invalid flatbuffer received from browser client.";
    return;
  }
  VLOG(1) << "Got a subscription message "
          << aos::FlatbufferToJson(&message.message());
  if (!message.message().has_channels_to_transfer()) {
    LOG(ERROR) << "No channels requested for transfer.";
    return;
  }

  // The client each time sends a full list of everything it wants to be
  // subscribed to.  It is our responsibility to remove channels which aren't
  // in that list and add ones which need to be.
  //
  // Start by clearing a tracking bit on each channel.  This takes O(number of
  // open channels), which should be small.
  //
  // Then open any new channels.  For any we visit which are already open,
  // don't update those.
  //
  // Finally, iterate over the channel list and purge anything which we didn't
  // touch.
  for (auto &it : channels_) {
    it.second.requested = false;
  }
  for (auto channel_request : *message.message().channels_to_transfer()) {
    const Channel *channel = channel_request->channel();
    if (channel == nullptr) {
      LOG(ERROR) << "Got unpopulated channel.";
      continue;
    }
    const TransferMethod transfer_method = channel_request->method();
    // Call GetChannel() before comparing the channel name/type to each
    // subscriber. This allows us to resolve any node or application
    // specific mappings.
    const Channel *comparison_channel =
        configuration::GetChannel(event_loop_->configuration(), channel,
                                  event_loop_->name(), event_loop_->node());
    if (comparison_channel == nullptr) {
      LOG(ERROR) << "Channel does not exist: "
                 << configuration::StrippedChannelToString(channel);
      continue;
    }
    if (!configuration::ChannelIsReadableOnNode(comparison_channel,
                                                event_loop_->node())) {
      LOG(ERROR) << "Channel not available on node "
                 << event_loop_->node()->name()->string_view() << ": "
                 << configuration::StrippedChannelToString(channel);
      continue;
    }

    size_t channel_index = configuration::ChannelIndex(
        event_loop_->configuration(), comparison_channel);

    auto it = channels_.find(channel_index);
    if (it == channels_.end()) {
      std::shared_ptr<ScopedDataChannel> data_channel =
          ScopedDataChannel::MakeDataChannel();

      std::weak_ptr<ScopedDataChannel> data_channel_weak_ptr = data_channel;

      data_channel->set_on_open([this, data_channel_weak_ptr, transfer_method,
                                 channel_index]() {
        std::shared_ptr<ScopedDataChannel> data_channel =
            data_channel_weak_ptr.lock();
        CHECK(data_channel) << ": Subscriber got destroyed before we started.";
        // Raw pointer inside the subscriber so we don't have a circular
        // reference.  AddListener will close it.
        subscribers_[channel_index]->AddListener(data_channel, transfer_method);
      });

      Subscriber *subscriber = subscribers_[channel_index].get();
      data_channel->set_on_close([subscriber, data_channel_weak_ptr]() {
        std::shared_ptr<ScopedDataChannel> data_channel =
            data_channel_weak_ptr.lock();
        CHECK(data_channel) << ": Subscriber got destroyed before we finished.";
        subscriber->RemoveListener(data_channel);
      });

      data_channel->Open(
          connection_.connection(),
          absl::StrCat(channel->name()->str(), "/", channel->type()->str()));

      auto pair = channels_.insert({channel_index, {data_channel, true}});
      it = pair.first;
    }

    it->second.requested = true;

    VLOG(1) << "Subscribe to: " << channel->type()->str();
  }

  for (auto &it : channels_) {
    if (!it.second.requested) {
      it.second.data_channel->Close();
      it.second.data_channel = nullptr;
    }
  }
}

}  // namespace web_proxy
}  // namespace aos
