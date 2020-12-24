#include "aos/network/web_proxy.h"

#include "aos/flatbuffer_merge.h"
#include "aos/network/connect_generated.h"
#include "aos/network/web_proxy_generated.h"
#include "aos/network/web_proxy_utils.h"
#include "aos/seasocks/seasocks_logger.h"
#include "api/create_peerconnection_factory.h"
#include "glog/logging.h"
#include "internal/Embedded.h"

namespace aos {
namespace web_proxy {

namespace {
// Based on webrtc examples. In our controlled environment we expect setting sdp
// to always succeed, and we can't do anything about a failure, so just ignore
// everything.
class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static DummySetSessionDescriptionObserver *Create() {
    return new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess() {}
  virtual void OnFailure(webrtc::RTCError /*error*/) {}
};

}  // namespace

WebsocketHandler::WebsocketHandler(::seasocks::Server *server,
                                   aos::EventLoop *event_loop, int buffer_size)
    : server_(server),
      config_(aos::CopyFlatBuffer(event_loop->configuration())),
      event_loop_(event_loop) {
  // We need to reference findEmbeddedContent() to make the linker happy...
  findEmbeddedContent("");
  const aos::Node *self = event_loop->node();

  for (uint i = 0; i < event_loop->configuration()->channels()->size(); ++i) {
    auto channel = event_loop->configuration()->channels()->Get(i);
    if (aos::configuration::ChannelIsReadableOnNode(channel, self)) {
      auto fetcher = event_loop->MakeRawFetcher(channel);
      subscribers_.emplace_back(std::make_unique<aos::web_proxy::Subscriber>(
          std::move(fetcher), i, buffer_size));
    }
  }
  TimerHandler *const timer = event_loop->AddTimer([this]() {
    for (auto &subscriber : subscribers_) {
      subscriber->RunIteration();
    }
  });

  event_loop->OnRun([timer, event_loop]() {
    timer->Setup(event_loop->monotonic_now(), std::chrono::milliseconds(100));
  });
}

void WebsocketHandler::onConnect(::seasocks::WebSocket *sock) {
  std::unique_ptr<Connection> conn = std::make_unique<Connection>(
      sock, server_, subscribers_, config_, event_loop_);
  connections_.insert({sock, std::move(conn)});
}

void WebsocketHandler::onData(::seasocks::WebSocket *sock, const uint8_t *data,
                              size_t size) {
  connections_[sock]->HandleWebSocketData(data, size);
}

void WebsocketHandler::onDisconnect(::seasocks::WebSocket *sock) {
  connections_.erase(sock);
}

WebProxy::WebProxy(aos::EventLoop *event_loop, int buffer_size)
    : WebProxy(event_loop, &internal_epoll_, buffer_size) {}

WebProxy::WebProxy(aos::ShmEventLoop *event_loop, int buffer_size)
    : WebProxy(event_loop, event_loop->epoll(), buffer_size) {}

WebProxy::WebProxy(aos::EventLoop *event_loop, aos::internal::EPoll *epoll,
                   int buffer_size)
    : epoll_(epoll),
      server_(std::make_shared<aos::seasocks::SeasocksLogger>(
          ::seasocks::Logger::Level::Info)),
      websocket_handler_(
          new WebsocketHandler(&server_, event_loop, buffer_size)) {
  server_.addWebSocketHandler("/ws", websocket_handler_);
  CHECK(server_.startListening(8080));

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
      timer->Setup(event_loop->monotonic_now(), std::chrono::milliseconds(10));
    });
  }
}

WebProxy::~WebProxy() {
  epoll_->DeleteFd(server_.fd());
  server_.terminate();
  CHECK(::seasocks::Server::PollResult::Terminated == server_.poll(0));
}

void Subscriber::RunIteration() {
  {
    // Manage updating the channels_ map given the pending_* members from the
    // *Listeners() methods.
    // We handle all the removals first so that we correctly handle the
    // situation where the user calls RemoveListener() and then AddListener()
    // between calls to RunIteration(). The reverse order (adding and then
    // removing before an update) is handled directly in RemoveListener() where
    // we remove things from the pending_channels_ map directly.
    MutexLocker lock(&mutex_);
    for (const auto &channel : pending_removal_) {
      channels_.erase(channel);
    }
    pending_removal_.clear();
    for (const auto &channel : pending_channels_) {
      channels_.insert(channel);
    }
    pending_channels_.clear();
  }
  if (channels_.empty() && buffer_size_ == 0) {
    return;
  }

  while (fetcher_->FetchNext()) {
    // If we aren't building up a buffer, short-circuit the FetchNext().
    if (buffer_size_ == 0) {
      fetcher_->Fetch();
    }
    Message message;
    message.index = fetcher_->context().queue_index;
    VLOG(2) << "Packing a message with " << GetPacketCount(fetcher_->context())
            << "packets";
    for (int packet_index = 0;
         packet_index < GetPacketCount(fetcher_->context()); ++packet_index) {
      flatbuffers::Offset<MessageHeader> message_offset =
          PackMessage(&fbb_, fetcher_->context(), channel_index_, packet_index);
      fbb_.Finish(message_offset);

      const flatbuffers::DetachedBuffer buffer = fbb_.Release();

      message.data.emplace_back(
          rtc::CopyOnWriteBuffer(buffer.data(), buffer.size()),
          true /* binary array */);
    }
    message_buffer_.push_back(std::move(message));
  }
  for (auto &conn : channels_) {
    rtc::scoped_refptr<webrtc::DataChannelInterface> rtc_channel = conn.first;
    ChannelInformation *channel_data = &conn.second;
    if (channel_data->transfer_method == TransferMethod::SUBSAMPLE) {
      SkipToLastMessage(channel_data);
    }
    const webrtc::DataBuffer *buffer = NextBuffer(channel_data);
    while (buffer != nullptr) {
      if (rtc_channel->buffered_amount() > 14000000) {
        VLOG(1) << "skipping a send because buffered amount is too high";
        break;
      }
      // Call Send() from the signalling thread. The Invoke() call blocks until
      // the handler has been called, so we do not need to handle any
      // synchronization on this end. The body of the handler should be kept as
      // short as possible to avoid blocking the signalling thread continuously
      // for any longer than necessary.
      channel_data->signaling_thread->Invoke<void>(
          RTC_FROM_HERE,
          [rtc_channel, buffer]() { rtc_channel->Send(*buffer); });
      buffer = NextBuffer(channel_data);
    }
  }
  if (buffer_size_ >= 0) {
    while (message_buffer_.size() > static_cast<size_t>(buffer_size_)) {
      message_buffer_.pop_front();
    }
  }
}

bool Subscriber::Compare(const Channel *channel) const {
  return channel->name()->string_view() ==
             fetcher_->channel()->name()->string_view() &&
         channel->type()->string_view() ==
             fetcher_->channel()->type()->string_view();
}

void Subscriber::AddListener(
    rtc::scoped_refptr<webrtc::DataChannelInterface> channel,
    TransferMethod transfer_method, rtc::Thread *signaling_thread) {
  MutexLocker lock(&mutex_);
  ChannelInformation info;
  info.transfer_method = transfer_method;
  info.signaling_thread = signaling_thread;
  pending_channels_.emplace(channel, info);
}

const webrtc::DataBuffer *Subscriber::NextBuffer(ChannelInformation *channel) {
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
    return &message_buffer_.front().data.at(0);
  }
  if (channel->current_queue_index > latest_index) {
    // We are still waiting on the next message to appear; return.
    return nullptr;
  }
  CHECK_EQ(latest_index - earliest_index + 1, message_buffer_.size())
      << "Inconsistent queue indices.";
  const size_t packets_in_message =
      message_buffer_[channel->current_queue_index - earliest_index]
          .data.size();
  CHECK_LT(0u, packets_in_message);
  CHECK_LT(channel->next_packet_number, packets_in_message);

  const webrtc::DataBuffer *data =
      &message_buffer_[channel->current_queue_index - earliest_index].data.at(
          channel->next_packet_number);

  ++channel->next_packet_number;
  if (channel->next_packet_number == packets_in_message) {
    ++channel->current_queue_index;
    channel->next_packet_number = 0;
  }

  return data;
}

void Subscriber::SkipToLastMessage(ChannelInformation *channel) {
  CHECK_NOTNULL(channel);
  if (message_buffer_.empty() ||
      channel->current_queue_index == message_buffer_.back().index) {
    return;
  }
  channel->current_queue_index = message_buffer_.back().index;
  channel->next_packet_number = 0;
}

void Subscriber::RemoveListener(
    rtc::scoped_refptr<webrtc::DataChannelInterface> channel) {
  MutexLocker lock(&mutex_);
  pending_channels_.erase(channel);
  pending_removal_.push_back(channel);
}

Connection::Connection(
    ::seasocks::WebSocket *sock, ::seasocks::Server *server,
    const std::vector<std::unique_ptr<Subscriber>> &subscribers,
    const aos::FlatbufferDetachedBuffer<aos::Configuration> &config,
    const EventLoop *event_loop)
    : sock_(sock),
      server_(server),
      subscribers_(subscribers),
      config_headers_(PackBuffer(config.span())),
      event_loop_(event_loop) {}

// Function called for web socket data. Parses the flatbuffer and
// handles it appropriately.
void Connection::HandleWebSocketData(const uint8_t *data, size_t size) {
  const FlatbufferSpan<WebSocketMessage> message({data, size});
  if (!message.Verify()) {
    LOG(ERROR) << "Invalid WebsocketMessage received from browser.";
    return;
  }
  switch (message.message().payload_type()) {
    case Payload::WebSocketSdp: {
      const WebSocketSdp *offer = message.message().payload_as_WebSocketSdp();
      if (offer->type() != SdpType::OFFER) {
        LOG(WARNING) << "Got the wrong sdp type from client";
        break;
      }
      const flatbuffers::String *sdp = offer->payload();
      webrtc::SdpParseError error;
      std::unique_ptr<webrtc::SessionDescriptionInterface> desc =
          CreateSessionDescription(webrtc::SdpType::kOffer, sdp->str(), &error);
      if (!desc) {
        LOG(WARNING) << "Failed to parse sdp description: "
                     << error.description;
        // TODO(alex): send a message back to browser for failure.
        break;
      }

      // We can only start creating the PeerConnection once we have
      // something to give it, so we wait until we get an offer before
      // starting.
      webrtc::PeerConnectionInterface::RTCConfiguration config;
      config.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
      config.enable_dtls_srtp = true;

      std::unique_ptr<rtc::Thread> signaling_thread = rtc::Thread::Create();
      signaling_thread->SetName("signaling_thread", nullptr);
      signaling_thread->Start();

      signaling_thread_ = signaling_thread.get();

      webrtc::PeerConnectionFactoryDependencies factory_deps;
      factory_deps.signaling_thread = signaling_thread.release();
      rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> factory =
          CreateModularPeerConnectionFactory(std::move(factory_deps));

      peer_connection_ =
          factory->CreatePeerConnection(config, nullptr, nullptr, this);

      peer_connection_->SetRemoteDescription(
          DummySetSessionDescriptionObserver::Create(), desc.release());

      peer_connection_->CreateAnswer(
          this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
      break;
    }
    case Payload::WebSocketIce: {
      const WebSocketIce *ice = message.message().payload_as_WebSocketIce();
      std::string candidate = ice->candidate()->str();
      std::string sdpMid = ice->sdpMid()->str();
      int sdpMLineIndex = ice->sdpMLineIndex();
      webrtc::SdpParseError error;
      webrtc::IceCandidateInterface *ice_candidate =
          webrtc::CreateIceCandidate(sdpMid, sdpMLineIndex, candidate, &error);
      if (!ice_candidate) {
        LOG(WARNING) << "Failed to parse ice candidate: " << error.description;
        // TODO(alex): send a message back to browser for failure.
        break;
      }
      peer_connection_->AddIceCandidate(ice_candidate);
      break;
    }
    default: {
      break;
    }
  }
}

void Connection::Send(const ::flatbuffers::DetachedBuffer &buffer) const {
  webrtc::DataBuffer data_buffer(
      rtc::CopyOnWriteBuffer(buffer.data(), buffer.size()),
      true /* binary array */);
  VLOG(1) << "Sending " << buffer.size() << "bytes to a client";
  data_channel_->Send(data_buffer);
}

void Connection::OnDataChannel(
    rtc::scoped_refptr<webrtc::DataChannelInterface> channel) {
  data_channel_ = channel;
  data_channel_->RegisterObserver(this);
}

void Connection::OnIceCandidate(
    const webrtc::IceCandidateInterface *candidate) {
  flatbuffers::FlatBufferBuilder fbb(512);
  std::string ice_string;
  candidate->ToString(&ice_string);

  flatbuffers::Offset<WebSocketIce> ice_fb = CreateWebSocketIceDirect(
      fbb, ice_string.c_str(), candidate->sdp_mid().c_str(),
      candidate->sdp_mline_index());
  flatbuffers::Offset<WebSocketMessage> ice_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketIce, ice_fb.Union());
  fbb.Finish(ice_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

// This is the callback for creating an sdp. We have to manually assign it
// locally and send it to the client.
void Connection::OnSuccess(webrtc::SessionDescriptionInterface *desc) {
  peer_connection_->SetLocalDescription(
      DummySetSessionDescriptionObserver::Create(), desc);
  flatbuffers::FlatBufferBuilder fbb(512);
  std::string answer_string;
  desc->ToString(&answer_string);
  flatbuffers::Offset<WebSocketSdp> sdp_fb =
      CreateWebSocketSdpDirect(fbb, SdpType::ANSWER, answer_string.c_str());
  flatbuffers::Offset<WebSocketMessage> answer_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketSdp, sdp_fb.Union());
  fbb.Finish(answer_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

// Wait until the data channel is ready for data before sending the config.
void Connection::OnStateChange() {
  if (peer_connection_.get() != nullptr &&
      data_channel_->state() == webrtc::DataChannelInterface::kOpen) {
    for (const auto &header : config_headers_) {
      Send(header.buffer());
    }
  }
}

// Handle DataChannel messages. Subscribe to each listener that matches the
// subscribe message
void Connection::OnMessage(const webrtc::DataBuffer &buffer) {
  // Sanity check--we are relying on the Add/RemoveListener calls being made
  // from the correct thread.
  CHECK(signaling_thread_->IsCurrent());
  FlatbufferSpan<SubscriberRequest> message(
      {buffer.data.data(), buffer.data.size()});
  if (!message.Verify()) {
    LOG(ERROR) << "Invalid flatbuffer received from browser client.";
    return;
  }
  VLOG(2) << "Got a subscription message "
          << aos::FlatbufferToJson(&message.message());
  if (!message.message().has_channels_to_transfer()) {
    LOG(ERROR) << "No channels requested for transfer.";
    return;
  }
  for (auto &subscriber : subscribers_) {
    bool found_match = false;
    for (auto channel_request : *message.message().channels_to_transfer()) {
      const Channel *channel = channel_request->channel();
      if (channel == nullptr) {
        LOG(ERROR) << "Got unpopulated channel.";
        continue;
      }
      const TransferMethod transfer_method = channel_request->method();
      // Call GetChannel() before comparing the channel name/type to each
      // subscriber. This allows us to resolve any node or application specific
      // mappings.
      const Channel *comparison_channel =
          configuration::GetChannel(event_loop_->configuration(), channel,
                                    event_loop_->name(), event_loop_->node());
      if (comparison_channel == nullptr) {
        LOG(ERROR) << "Channel not available: "
                   << configuration::StrippedChannelToString(channel);
        continue;
      }
      if (subscriber->Compare(comparison_channel)) {
        int index = subscriber->index();
        auto it = channels_.find(index);
        if (it == channels_.end()) {
          auto pair = channels_.insert(
              {index, peer_connection_->CreateDataChannel(
                          channel->name()->str() + "/" + channel->type()->str(),
                          nullptr)});
          it = pair.first;
        }
        subscriber->AddListener(it->second, transfer_method, signaling_thread_);

        VLOG(1) << "Subscribe to: " << channel->type()->str();
        found_match = true;
        break;
      }
    }
    if (!found_match) {
      int index = subscriber->index();
      auto it = channels_.find(index);
      subscriber->RemoveListener(it->second);
    }
  }
}

}  // namespace web_proxy
}  // namespace aos
