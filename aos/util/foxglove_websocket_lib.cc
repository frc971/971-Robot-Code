#include "aos/util/foxglove_websocket_lib.h"

#include "aos/util/mcap_logger.h"
#include "aos/flatbuffer_merge.h"
#include "absl/strings/escaping.h"
#include "gflags/gflags.h"

DEFINE_uint32(sorting_buffer_ms, 100,
              "Amount of time to buffer messages to sort them before sending "
              "them to foxglove.");

namespace {
// Period at which to poll the fetchers for all the channels.
constexpr std::chrono::milliseconds kPollPeriod{50};
}  // namespace

namespace aos {
FoxgloveWebsocketServer::FoxgloveWebsocketServer(
    aos::EventLoop *event_loop, uint32_t port, Serialization serialization,
    FetchPinnedChannels fetch_pinned_channels)
    : event_loop_(event_loop),
      serialization_(serialization),
      fetch_pinned_channels_(fetch_pinned_channels),
      server_(port, "aos_foxglove") {
  for (const aos::Channel *channel :
       *event_loop_->configuration()->channels()) {
    const bool is_pinned = (channel->read_method() == ReadMethod::PIN);
    if (aos::configuration::ChannelIsReadableOnNode(channel,
                                                    event_loop_->node()) &&
        (!is_pinned || fetch_pinned_channels_ == FetchPinnedChannels::kYes)) {
      const FlatbufferDetachedBuffer<reflection::Schema> schema =
          RecursiveCopyFlatBuffer(channel->schema());
      const ChannelId id =
          (serialization_ == Serialization::kJson)
              ? server_.addChannel(foxglove::websocket::ChannelWithoutId{
                    .topic =
                        channel->name()->str() + " " + channel->type()->str(),
                    .encoding = "json",
                    .schemaName = channel->type()->str(),
                    .schema =
                        JsonSchemaForFlatbuffer({channel->schema()}).dump()})
              : server_.addChannel(foxglove::websocket::ChannelWithoutId{
                    .topic =
                        channel->name()->str() + " " + channel->type()->str(),
                    .encoding = "flatbuffer",
                    .schemaName = channel->type()->str(),
                    .schema = absl::Base64Escape(
                        {reinterpret_cast<const char *>(schema.span().data()),
                         schema.span().size()})});
      CHECK(fetchers_.count(id) == 0);
      fetchers_[id] =
          FetcherState{.fetcher = event_loop_->MakeRawFetcher(channel)};
    }
  }

  server_.setSubscribeHandler([this](ChannelId channel) {
    if (fetchers_.count(channel) == 0) {
      return;
    }
    if (active_channels_.count(channel) == 0) {
      // Catch up to the latest message on the requested channel, then subscribe
      // to it.
      fetchers_[channel].fetcher->Fetch();
      active_channels_.insert(channel);
    }
  });
  server_.setUnsubscribeHandler(
      [this](ChannelId channel) { active_channels_.erase(channel); });
  aos::TimerHandler *timer = event_loop_->AddTimer([this]() {
    // In order to run the websocket server, we just let it spin every cycle for
    // a bit. This isn't great for integration, but lets us stay in control and
    // until we either have (a) a chance to locate a file descriptor to hand
    // epoll; or (b) rewrite the foxglove websocket server to use seasocks
    // (which we know how to integrate), we'll just function with this.
    // TODO(james): Tighter integration into our event loop structure.
    server_.run_for(kPollPeriod / 2);

    // Unfortunately, we can't just push out all the messages as they come in.
    // Foxglove expects that the timestamps associated with each message to be
    // monotonic, and if you send things out of order then it will clear the
    // state of the visualization entirely, which makes viewing plots
    // impossible. If the user only accesses a single channel, that is fine, but
    // as soon as they try to use multiple channels, you encounter interleaving.
    // To resolve this, we specify a buffer (--sorting_buffer_ms), and only send
    // out messages older than that time, sorting everything before we send it
    // out.
    const aos::monotonic_clock::time_point sort_until =
        event_loop_->monotonic_now() -
        std::chrono::milliseconds(FLAGS_sorting_buffer_ms);

    // Pair of <send_time, channel id>.
    absl::btree_set<std::pair<aos::monotonic_clock::time_point, ChannelId>>
        fetcher_times;

    // Go through and seed fetcher_times with the first message on each channel.
    for (const ChannelId channel : active_channels_) {
      FetcherState *fetcher = &fetchers_[channel];
      if (fetcher->sent_current_message) {
        if (fetcher->fetcher->FetchNext()) {
          fetcher->sent_current_message = false;
        }
      }
      if (!fetcher->sent_current_message) {
        const aos::monotonic_clock::time_point send_time =
            fetcher->fetcher->context().monotonic_event_time;
        if (send_time <= sort_until) {
          fetcher_times.insert(std::make_pair(send_time, channel));
        }
      }
    }

    // Send the oldest message continually until we run out of messages to send.
    while (!fetcher_times.empty()) {
      const ChannelId channel = fetcher_times.begin()->second;
      FetcherState *fetcher = &fetchers_[channel];
      switch (serialization_) {
        case Serialization::kJson:
          server_.sendMessage(
              channel, fetcher_times.begin()->first.time_since_epoch().count(),
              aos::FlatbufferToJson(fetcher->fetcher->channel()->schema(),
                                    static_cast<const uint8_t *>(
                                        fetcher->fetcher->context().data)));
          break;
        case Serialization::kFlatbuffer:
          server_.sendMessage(
              channel, fetcher_times.begin()->first.time_since_epoch().count(),
              {static_cast<const char *>(fetcher->fetcher->context().data),
               fetcher->fetcher->context().size});
      }
      fetcher_times.erase(fetcher_times.begin());
      fetcher->sent_current_message = true;
      if (fetcher->fetcher->FetchNext()) {
        fetcher->sent_current_message = false;
        const aos::monotonic_clock::time_point send_time =
            fetcher->fetcher->context().monotonic_event_time;
        if (send_time <= sort_until) {
          fetcher_times.insert(std::make_pair(send_time, channel));
        }
      }
    }
  });

  event_loop_->OnRun([timer, this]() {
    timer->Setup(event_loop_->monotonic_now(), kPollPeriod);
  });
}
FoxgloveWebsocketServer::~FoxgloveWebsocketServer() { server_.stop(); }
}  // namespace aos
