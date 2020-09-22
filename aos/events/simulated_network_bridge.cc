#include "aos/events/simulated_network_bridge.h"

#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"

namespace aos {
namespace message_bridge {

// This class delays messages forwarded between two factories.
//
// The basic design is that we need to use the distributed_clock to convert
// monotonic times from the source to the destination node.  We also use a
// fetcher to manage the queue of data, and a timer to schedule the sends.
class RawMessageDelayer {
 public:
  RawMessageDelayer(aos::NodeEventLoopFactory *fetch_node_factory,
                    aos::NodeEventLoopFactory *send_node_factory,
                    aos::EventLoop *send_event_loop,
                    std::unique_ptr<aos::RawFetcher> fetcher,
                    std::unique_ptr<aos::RawSender> sender,
                    ServerConnection *server_connection, int client_index,
                    MessageBridgeClientStatus *client_status,
                    size_t channel_index,
                    aos::Sender<logger::MessageHeader> *timestamp_logger)
      : fetch_node_factory_(fetch_node_factory),
        send_node_factory_(send_node_factory),
        send_event_loop_(send_event_loop),
        fetcher_(std::move(fetcher)),
        sender_(std::move(sender)),
        server_connection_(server_connection),
        client_status_(client_status),
        client_index_(client_index),
        client_connection_(client_status_->GetClientConnection(client_index)),
        channel_index_(channel_index),
        timestamp_logger_(timestamp_logger) {
    timer_ = send_event_loop_->AddTimer([this]() { Send(); });

    Schedule();
  }

  const Channel *channel() const { return fetcher_->channel(); }

  // Kicks us to re-fetch and schedule the timer.
  void Schedule() {
    // Keep pulling messages out of the fetcher until we find one in the future.
    while (true) {
      if (fetcher_->context().data == nullptr || sent_) {
        sent_ = !fetcher_->FetchNext();
      }
      if (sent_) {
        break;
      }
      if (fetcher_->context().monotonic_event_time +
              send_node_factory_->network_delay() +
              send_node_factory_->send_delay() >
          fetch_node_factory_->monotonic_now()) {
        break;
      }

      // TODO(austin): Not cool.  We want to actually forward these.  This means
      // we need a more sophisticated concept of what is running.
      LOG(WARNING) << "Not forwarding message on "
                   << configuration::CleanedChannelToString(fetcher_->channel())
                   << " because we aren't running.  Set at "
                   << fetcher_->context().monotonic_event_time << " now is "
                   << fetch_node_factory_->monotonic_now();
      sent_ = true;
      server_connection_->mutate_dropped_packets(
          server_connection_->dropped_packets() + 1);
    }

    if (fetcher_->context().data == nullptr) {
      return;
    }

    if (sent_) {
      return;
    }

    // Compute the time to publish this message.
    const monotonic_clock::time_point monotonic_delivered_time =
        DeliveredTime(fetcher_->context());

    CHECK_GE(monotonic_delivered_time, send_node_factory_->monotonic_now())
        << ": Trying to deliver message in the past on channel "
        << configuration::StrippedChannelToString(fetcher_->channel())
        << " to node " << send_event_loop_->node()->name()->string_view()
        << " sent from " << fetcher_->channel()->source_node()->string_view()
        << " at " << fetch_node_factory_->monotonic_now();

    server_connection_->mutate_sent_packets(server_connection_->sent_packets() +
                                            1);
    timer_->Setup(monotonic_delivered_time);
  }

 private:
  // Acutally sends the message, and reschedules.
  void Send() {
    // Fill out the send times.
    sender_->Send(fetcher_->context().data, fetcher_->context().size,
                  fetcher_->context().monotonic_event_time,
                  fetcher_->context().realtime_event_time,
                  fetcher_->context().queue_index);

    // And simulate message_bridge's offset recovery.
    client_status_->SampleFilter(client_index_,
                                 fetcher_->context().monotonic_event_time,
                                 sender_->monotonic_sent_time());

    client_connection_->mutate_received_packets(
        client_connection_->received_packets() + 1);

    if (timestamp_logger_) {
      aos::Sender<logger::MessageHeader>::Builder builder =
          timestamp_logger_->MakeBuilder();

      logger::MessageHeader::Builder message_header_builder =
          builder.MakeBuilder<logger::MessageHeader>();

      message_header_builder.add_channel_index(channel_index_);

      // Swap the remote and sent metrics.  They are from the sender's
      // perspective, not the receiver's perspective.
      message_header_builder.add_monotonic_remote_time(
          fetcher_->context().monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          fetcher_->context().realtime_event_time.time_since_epoch().count());
      message_header_builder.add_remote_queue_index(
          fetcher_->context().queue_index);

      message_header_builder.add_monotonic_sent_time(
          sender_->monotonic_sent_time().time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          sender_->realtime_sent_time().time_since_epoch().count());
      message_header_builder.add_queue_index(sender_->sent_queue_index());

      builder.Send(message_header_builder.Finish());
    }

    sent_ = true;
    Schedule();
  }

  // Converts from time on the sending node to time on the receiving node.
  monotonic_clock::time_point DeliveredTime(const Context &context) const {
    const distributed_clock::time_point distributed_sent_time =
        fetch_node_factory_->ToDistributedClock(context.monotonic_event_time);

    return send_node_factory_->FromDistributedClock(
        distributed_sent_time + send_node_factory_->network_delay() +
        send_node_factory_->send_delay());
  }

  // Factories used for time conversion.
  aos::NodeEventLoopFactory *fetch_node_factory_;
  aos::NodeEventLoopFactory *send_node_factory_;

  // Event loop which sending is scheduled on.
  aos::EventLoop *send_event_loop_;
  // Timer used to send.
  aos::TimerHandler *timer_;
  // Fetcher used to receive messages.
  std::unique_ptr<aos::RawFetcher> fetcher_;
  // Sender to send them back out.
  std::unique_ptr<aos::RawSender> sender_;
  // True if we have sent the message in the fetcher.
  bool sent_ = false;

  ServerConnection *server_connection_ = nullptr;
  MessageBridgeClientStatus *client_status_ = nullptr;
  int client_index_;
  ClientConnection *client_connection_ = nullptr;

  size_t channel_index_;
  aos::Sender<logger::MessageHeader> *timestamp_logger_ = nullptr;
};

SimulatedMessageBridge::SimulatedMessageBridge(
    SimulatedEventLoopFactory *simulated_event_loop_factory) {
  CHECK(
      configuration::MultiNode(simulated_event_loop_factory->configuration()));

  // Pre-build up event loops for every node.  They are pretty cheap anyways.
  for (const Node *node : simulated_event_loop_factory->nodes()) {
    auto it = event_loop_map_.emplace(std::make_pair(
        node,
        simulated_event_loop_factory->MakeEventLoop("message_bridge", node)));

    CHECK(it.second);

    it.first->second.event_loop->SkipTimingReport();
    it.first->second.event_loop->SkipAosLog();

    for (ServerConnection *connection :
         it.first->second.server_status.server_connection()) {
      if (connection == nullptr) continue;

      connection->mutate_state(message_bridge::State::CONNECTED);
    }

    for (size_t i = 0;
         i < it.first->second.client_status.mutable_client_statistics()
                 ->mutable_connections()
                 ->size();
         ++i) {
      ClientConnection *connection =
          it.first->second.client_status.mutable_client_statistics()
              ->mutable_connections()
              ->GetMutableObject(i);
      if (connection == nullptr) continue;

      connection->mutate_state(message_bridge::State::CONNECTED);
    }
  }

  for (const Channel *channel :
       *simulated_event_loop_factory->configuration()->channels()) {
    if (!channel->has_destination_nodes()) {
      continue;
    }

    // Find the sending node.
    const Node *node =
        configuration::GetNode(simulated_event_loop_factory->configuration(),
                               channel->source_node()->string_view());
    auto source_event_loop = event_loop_map_.find(node);
    CHECK(source_event_loop != event_loop_map_.end());

    std::unique_ptr<DelayersVector> delayers =
        std::make_unique<DelayersVector>();

    // And then build up a RawMessageDelayer for each destination.
    for (const Connection *connection : *channel->destination_nodes()) {
      const Node *destination_node =
          configuration::GetNode(simulated_event_loop_factory->configuration(),
                                 connection->name()->string_view());
      auto destination_event_loop = event_loop_map_.find(destination_node);
      CHECK(destination_event_loop != event_loop_map_.end());

      ServerConnection *server_connection =
          source_event_loop->second.server_status.FindServerConnection(
              connection->name()->string_view());

      int client_index =
          destination_event_loop->second.client_status.FindClientIndex(
              channel->source_node()->string_view());

      const size_t destination_node_index = configuration::GetNodeIndex(
          simulated_event_loop_factory->configuration(), destination_node);

      const bool delivery_time_is_logged =
          configuration::ConnectionDeliveryTimeIsLoggedOnNode(
              connection, source_event_loop->second.event_loop->node());

      delayers->emplace_back(std::make_unique<RawMessageDelayer>(
          simulated_event_loop_factory->GetNodeEventLoopFactory(node),
          simulated_event_loop_factory->GetNodeEventLoopFactory(
              destination_node),
          destination_event_loop->second.event_loop.get(),
          source_event_loop->second.event_loop->MakeRawFetcher(channel),
          destination_event_loop->second.event_loop->MakeRawSender(channel),
          server_connection, client_index,
          &destination_event_loop->second.client_status,
          configuration::ChannelIndex(
              source_event_loop->second.event_loop->configuration(), channel),
          delivery_time_is_logged
              ? &source_event_loop->second
                     .timestamp_loggers[destination_node_index]
              : nullptr));
    }

    const Channel *const timestamp_channel = configuration::GetChannel(
        simulated_event_loop_factory->configuration(), "/aos",
        Timestamp::GetFullyQualifiedName(),
        source_event_loop->second.event_loop->name(), node);

    if (channel == timestamp_channel) {
      source_event_loop->second.server_status.set_send_data(
          [captured_delayers = delayers.get()](const Context &) {
            for (std::unique_ptr<RawMessageDelayer> &delayer :
                 *captured_delayers) {
              delayer->Schedule();
            }
          });
    } else {
      // And register every delayer to be poked when a new message shows up.
      source_event_loop->second.event_loop->MakeRawNoArgWatcher(
          channel, [captured_delayers = delayers.get()](const Context &) {
            for (std::unique_ptr<RawMessageDelayer> &delayer :
                 *captured_delayers) {
              delayer->Schedule();
            }
          });
    }
    delayers_list_.emplace_back(std::move(delayers));
  }
}

SimulatedMessageBridge::~SimulatedMessageBridge() {}

void SimulatedMessageBridge::DisableForwarding(const Channel *channel) {
  for (std::unique_ptr<std::vector<std::unique_ptr<RawMessageDelayer>>>
           &delayers : delayers_list_) {
    if (delayers->size() > 0) {
      if ((*delayers)[0]->channel() == channel) {
        for (std::unique_ptr<RawMessageDelayer> &delayer : *delayers) {
          CHECK(delayer->channel() == channel);
        }

        // If we clear the delayers list, nothing will be scheduled.  Which is a
        // success!
        delayers->clear();
      }
    }
  }
}

void SimulatedMessageBridge::DisableStatistics() {
  for (std::pair<const Node *const, State> &state : event_loop_map_) {
    state.second.server_status.DisableStatistics();
    state.second.client_status.DisableStatistics();
  }
}

SimulatedMessageBridge::State::State(
    std::unique_ptr<aos::EventLoop> &&new_event_loop)
    : event_loop(std::move(new_event_loop)),
      server_status(event_loop.get()),
      client_status(event_loop.get()) {
  timestamp_loggers.resize(event_loop->configuration()->nodes()->size());

  // Find all nodes which log timestamps back to us (from us).
  for (const Channel *channel : *event_loop->configuration()->channels()) {
    CHECK(channel->has_source_node());

    // Sent by us.
    if (configuration::ChannelIsSendableOnNode(channel, event_loop->node()) &&
        channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        const bool delivery_time_is_logged =
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                connection, event_loop->node());

        // And the timestamps are then logged back by us again.
        if (!delivery_time_is_logged) {
          continue;
        }

        // (And only construct the sender if it hasn't been constructed)
        const Node *other_node = configuration::GetNode(
            event_loop->configuration(), connection->name()->string_view());
        const size_t other_node_index = configuration::GetNodeIndex(
            event_loop->configuration(), other_node);

        if (!timestamp_loggers[other_node_index]) {
          timestamp_loggers[other_node_index] =
              event_loop->MakeSender<logger::MessageHeader>(
                  absl::StrCat("/aos/remote_timestamps/",
                               connection->name()->string_view()));
        }
      }
    }
  }
}

}  // namespace message_bridge
}  // namespace aos
