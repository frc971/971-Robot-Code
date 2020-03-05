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
                    std::unique_ptr<aos::RawSender> sender)
      : fetch_node_factory_(fetch_node_factory),
        send_node_factory_(send_node_factory),
        send_event_loop_(send_event_loop),
        fetcher_(std::move(fetcher)),
        sender_(std::move(sender)) {
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
        << ": Trying to deliver message in the past...";

    timer_->Setup(monotonic_delivered_time);
  }

 private:
  // Acutally sends the message, and reschedules.
  void Send() {
    // Compute the time to publish this message.
    const monotonic_clock::time_point monotonic_delivered_time =
        DeliveredTime(fetcher_->context());

    CHECK_EQ(monotonic_delivered_time, send_node_factory_->monotonic_now())
        << ": Message to be sent at the wrong time.";

    // And also fill out the send times as well.
    sender_->Send(fetcher_->context().data, fetcher_->context().size,
                  fetcher_->context().monotonic_event_time,
                  fetcher_->context().realtime_event_time,
                  fetcher_->context().queue_index);

    sent_ = true;
    Schedule();
  }

  // Converts from time on the sending node to time on the receiving node.
  monotonic_clock::time_point DeliveredTime(const Context &context) const {
    const distributed_clock::time_point distributed_sent_time =
        fetch_node_factory_->ToDistributedClock(context.monotonic_event_time);

    return aos::monotonic_clock::epoch() +
           (distributed_sent_time - send_node_factory_->ToDistributedClock(
                                        aos::monotonic_clock::epoch())) +
           send_node_factory_->network_delay() +
           send_node_factory_->send_delay();
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
};

SimulatedMessageBridge::SimulatedMessageBridge(
    SimulatedEventLoopFactory *simulated_event_loop_factory) {
  CHECK(
      configuration::MultiNode(simulated_event_loop_factory->configuration()));

  // Pre-build up event loops for every node.  They are pretty cheap anyways.
  for (const Node *node : simulated_event_loop_factory->nodes()) {
    auto it = event_loop_map_.insert(
        {node,
         simulated_event_loop_factory->MakeEventLoop("message_bridge", node)});

    CHECK(it.second);

    it.first->second->SkipTimingReport();
    it.first->second->SkipAosLog();
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

      delayers->emplace_back(std::make_unique<RawMessageDelayer>(
          simulated_event_loop_factory->GetNodeEventLoopFactory(node),
          simulated_event_loop_factory->GetNodeEventLoopFactory(
              destination_node),
          destination_event_loop->second.get(),
          source_event_loop->second->MakeRawFetcher(channel),
          destination_event_loop->second->MakeRawSender(channel)));
    }

    // And register every delayer to be poked when a new message shows up.
    source_event_loop->second->MakeRawNoArgWatcher(
        channel, [captured_delayers = delayers.get()](const Context &) {
          for (std::unique_ptr<RawMessageDelayer> &delayer :
               *captured_delayers) {
            delayer->Schedule();
          }
        });
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

}  // namespace message_bridge
}  // namespace aos
