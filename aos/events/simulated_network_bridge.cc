#include "aos/events/simulated_network_bridge.h"

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/network/remote_message_generated.h"

namespace aos::message_bridge {

// This class delays messages forwarded between two factories.
//
// The basic design is that we need to use the distributed_clock to convert
// monotonic times from the source to the destination node.  We also use a
// fetcher to manage the queue of data, and a timer to schedule the sends.
class RawMessageDelayer {
 public:
  RawMessageDelayer(const Channel *channel, const Connection *connection,
                    aos::NodeEventLoopFactory *fetch_node_factory,
                    aos::NodeEventLoopFactory *send_node_factory,
                    size_t destination_node_index, bool delivery_time_is_logged)
      : channel_(channel),
        connection_(connection),
        fetch_node_factory_(fetch_node_factory),
        send_node_factory_(send_node_factory),
        destination_node_index_(destination_node_index),
        channel_index_(configuration::ChannelIndex(
            fetch_node_factory_->configuration(), channel_)),
        delivery_time_is_logged_(delivery_time_is_logged) {}

  bool forwarding_disabled() const { return forwarding_disabled_; }
  void set_forwarding_disabled(bool forwarding_disabled) {
    forwarding_disabled_ = forwarding_disabled;
    if (!forwarding_disabled_) {
      CHECK(timestamp_logger_ == nullptr);
      CHECK(sender_ == nullptr);
    }
  }

  void SetFetchEventLoop(aos::EventLoop *fetch_event_loop,
                         MessageBridgeServerStatus *server_status,
                         ChannelTimestampSender *timestamp_loggers) {
    sent_ = false;
    reliable_scheduled_ = false;
    published_ = false;
    fetch_event_loop_ = fetch_event_loop;
    if (fetch_event_loop_) {
      fetcher_ = fetch_event_loop_->MakeRawFetcher(channel_);
    } else {
      fetcher_ = nullptr;
    }

    server_status_ = server_status;
    if (server_status_) {
      server_connection_ =
          server_status_->FindServerConnection(send_node_factory_->node());
      server_index_ = configuration::GetNodeIndex(
          send_node_factory_->configuration(), send_node_factory_->node());
    }
    if (delivery_time_is_logged_ && timestamp_loggers != nullptr &&
        !forwarding_disabled_) {
      timestamp_logger_ =
          timestamp_loggers->SenderForChannel(channel_, connection_);
    } else {
      timestamp_logger_ = nullptr;
    }

    if (fetch_event_loop_) {
      timestamp_timer_ =
          fetch_event_loop_->AddTimer([this]() { SendTimestamp(); });
      if (send_event_loop_) {
        std::string timer_name =
            absl::StrCat(send_event_loop_->node()->name()->string_view(), " ",
                         fetcher_->channel()->name()->string_view(), " ",
                         fetcher_->channel()->type()->string_view());
        if (timer_) {
          timer_->set_name(timer_name);
        }
        timestamp_timer_->set_name(absl::StrCat(timer_name, " timestamps"));
      }
    } else {
      timestamp_timer_ = nullptr;
    }
  }

  void SetSendEventLoop(aos::EventLoop *send_event_loop,
                        MessageBridgeClientStatus *client_status) {
    send_event_loop_ = send_event_loop;
    if (send_event_loop_ && !forwarding_disabled_) {
      sender_ = send_event_loop_->MakeRawSender(channel_);
    } else {
      sender_ = nullptr;
    }

    client_status_ = client_status;
    if (client_status_) {
      client_index_ = client_status_->FindClientIndex(
          channel_->source_node()->string_view());
      client_connection_ = client_status_->GetClientConnection(client_index_);
    } else {
      client_index_ = -1;
      client_connection_ = nullptr;
    }

    if (send_event_loop_) {
      timer_ = send_event_loop_->AddTimer([this]() { Send(); });
      if (fetcher_) {
        std::string timer_name =
            absl::StrCat(send_event_loop_->node()->name()->string_view(), " ",
                         fetcher_->channel()->name()->string_view(), " ",
                         fetcher_->channel()->type()->string_view());
        timer_->set_name(timer_name);
        if (timestamp_timer_) {
          timestamp_timer_->set_name(absl::StrCat(timer_name, " timestamps"));
        }
      }
    } else {
      timer_ = nullptr;
    }
  }

  const Channel *channel() const { return channel_; }

  uint32_t time_to_live() {
    return configuration::ConnectionToNode(channel_, send_node_factory_->node())
        ->time_to_live();
  }

  std::string Name() {
    std::string result;
    result +=
        (fetch_event_loop_ ? fetch_event_loop_->node()->name()->string_view()
                           : std::string_view("?"));
    result += " -> ";
    result +=
        (send_event_loop_ ? send_event_loop_->node()->name()->string_view()
                          : std::string_view("?"));
    result += " ";
    result += aos::configuration::StrippedChannelToString(channel());
    return result;
  }

  void ScheduleReliable() {
    if (forwarding_disabled()) {
      return;
    }

    if (!fetcher_) {
      return;
    }
    if (fetcher_->context().data == nullptr || sent_) {
      fetcher_->Fetch();
      sent_ = fetcher_->context().data == nullptr;
      published_ = sent_;
      reliable_scheduled_ = true;
    }

    if (!timer_) {
      return;
    }

    if (server_connection_->state() != State::CONNECTED) {
      reliable_scheduled_ = false;
      sent_ = true;
      return;
    }

    FetchNext();
    if (fetcher_->context().data == nullptr || sent_) {
      return;
    }

    // Send at startup.  It is the best we can do.
    const monotonic_clock::time_point monotonic_delivered_time =
        send_node_factory_->monotonic_now() +
        send_node_factory_->network_delay();

    CHECK_GE(monotonic_delivered_time, send_node_factory_->monotonic_now())
        << ": Trying to deliver message in the past on channel "
        << configuration::StrippedChannelToString(fetcher_->channel())
        << " to node " << send_event_loop_->node()->name()->string_view()
        << " sent from " << fetcher_->channel()->source_node()->string_view()
        << " at " << fetch_node_factory_->monotonic_now();

    if (!timer_scheduled_) {
      server_status_->AddSentPacket(server_index_, channel_);
      timer_->Schedule(monotonic_delivered_time);
      timer_scheduled_ = true;

      QueueTransmitTimestamp(fetcher_->context().queue_index,
                             fetcher_->context().monotonic_event_time,
                             fetch_event_loop_->monotonic_now());
    }
  }

  bool timer_scheduled_ = false;

  void MessageWatcherCallback(uint32_t sent_queue_index,
                              monotonic_clock::time_point monotonic_sent_time,
                              monotonic_clock::time_point transmit_time) {
    if (!reliable_scheduled_) {
      QueueTransmitTimestamp(sent_queue_index, monotonic_sent_time,
                             transmit_time);
    } else {
      reliable_scheduled_ = false;
    }
    Schedule();
  }

  void QueueTransmitTimestamp(uint32_t sent_queue_index,
                              monotonic_clock::time_point monotonic_sent_time,
                              monotonic_clock::time_point transmit_time) {
    if (forwarding_disabled()) return;

    if (monotonic_remote_transmit_times_.size() > 0u) {
      // FetchNext can discover messages before we do in the same nanosecond. In
      // that case, make sure the contents match and don't add it a second time.
      auto back = monotonic_remote_transmit_times_
          [monotonic_remote_transmit_times_.size() - 1];
      if (back.sent_queue_index == sent_queue_index) {
        CHECK_EQ(back.monotonic_sent_time, monotonic_sent_time) << this;
        CHECK_EQ(back.transmit_time, transmit_time) << this;
        return;
      }
    }

    // Capture the time this message was published over the network on the
    // remote node
    monotonic_remote_transmit_times_.push_back(TransmitTime{
        .monotonic_sent_time = monotonic_sent_time,
        .sent_queue_index = sent_queue_index,
        .transmit_time = transmit_time,
    });
  }

  void Connect() {
    if (time_to_live() == 0 && published_ == false) {
      if (forwarding_disabled()) {
        return;
      }
      CHECK(fetcher_);

      fetcher_->Fetch();
      sent_ = fetcher_->context().data == nullptr;
      reliable_scheduled_ = true;

      QueueTransmitTimestamp(fetcher_->context().queue_index,
                             fetcher_->context().monotonic_event_time,
                             fetch_event_loop_->monotonic_now());
      Schedule();
    }
  }

  bool SendingTo(const Node *destination) {
    return send_event_loop_ && send_event_loop_->node() == destination;
  }

  // Kicks us to re-fetch and schedule the timer.
  void Schedule() {
    CHECK(!forwarding_disabled());
    if (!fetcher_) {
      return;
    }
    if (timer_scheduled_) {
      return;
    }
    FetchNext();
    if (fetcher_->context().data == nullptr || sent_) {
      return;
    }

    CHECK_GT(monotonic_remote_transmit_times_.size(), 0u) << this;
    const monotonic_clock::time_point transmit_time =
        monotonic_remote_transmit_times_[0].transmit_time;

    // Compute the time to publish this message.
    const monotonic_clock::time_point monotonic_delivered_time =
        DeliveredTime(transmit_time);

    CHECK_GE(monotonic_delivered_time, send_node_factory_->monotonic_now())
        << ": Trying to deliver message in the past on channel "
        << configuration::StrippedChannelToString(fetcher_->channel())
        << " to node " << send_event_loop_->node()->name()->string_view()
        << " sent from " << fetcher_->channel()->source_node()->string_view()
        << " at " << fetch_node_factory_->monotonic_now();

    CHECK(timer_);
    server_status_->AddSentPacket(server_index_, channel_);
    timer_->Schedule(monotonic_delivered_time);
    timer_scheduled_ = true;
  }

 private:
  void FetchNext() {
    CHECK(server_connection_);
    // Keep pulling messages out of the fetcher until we find one in the future.
    while (true) {
      if (fetcher_->context().data == nullptr || sent_) {
        sent_ = !fetcher_->FetchNext();
        if (!sent_) {
          published_ = false;
        }
        if (!sent_) {
          if (monotonic_remote_transmit_times_.size() == 0u) {
            QueueTransmitTimestamp(fetcher_->context().queue_index,
                                   fetcher_->context().monotonic_event_time,
                                   fetch_event_loop_->monotonic_now());
          }
        }
      }
      if (sent_) {
        break;
      }

      if (server_connection_->state() != State::CONNECTED) {
        CHECK_GT(monotonic_remote_transmit_times_.size(), 0u) << this;
        CHECK_EQ(monotonic_remote_transmit_times_[0].monotonic_sent_time,
                 fetcher_->context().monotonic_event_time)
            << this << " " << Name();
        CHECK_EQ(monotonic_remote_transmit_times_[0].sent_queue_index,
                 fetcher_->context().queue_index)
            << this << " " << Name();

        monotonic_remote_transmit_times_.erase(
            monotonic_remote_transmit_times_.begin());
        sent_ = true;
        reliable_scheduled_ = false;
        published_ = false;
        server_status_->AddDroppedPacket(server_index_, channel_);
        continue;
      }

      if (fetcher_->context().monotonic_event_time +
                  send_node_factory_->network_delay() +
                  send_node_factory_->send_delay() >=
              fetch_node_factory_->monotonic_now() ||
          time_to_live() == 0) {
        break;
      }

      // TODO(austin): Not cool.  We want to actually forward these.  This means
      // we need a more sophisticated concept of what is running.
      LOG(WARNING) << "Not forwarding message on "
                   << configuration::CleanedChannelToString(fetcher_->channel())
                   << " because we aren't running.  Sent at "
                   << fetcher_->context().monotonic_event_time << " now is "
                   << fetch_node_factory_->monotonic_now();
      sent_ = true;
      reliable_scheduled_ = false;
      published_ = false;
      server_status_->AddDroppedPacket(server_index_, channel_);
    }
  }

  // Actually sends the message, and reschedules.
  void Send() {
    timer_scheduled_ = false;
    CHECK(sender_);
    CHECK(client_status_);

    // Confirm that the first element in the times list is ours, and pull the
    // transmit time out of it.
    CHECK(!monotonic_remote_transmit_times_.empty());
    CHECK_EQ(monotonic_remote_transmit_times_[0].monotonic_sent_time,
             fetcher_->context().monotonic_event_time);
    CHECK_EQ(monotonic_remote_transmit_times_[0].sent_queue_index,
             fetcher_->context().queue_index);

    const monotonic_clock::time_point monotonic_remote_transmit_time =
        monotonic_remote_transmit_times_[0].transmit_time;

    monotonic_remote_transmit_times_.erase(
        monotonic_remote_transmit_times_.begin());

    if (server_connection_->state() != State::CONNECTED) {
      sent_ = true;
      reliable_scheduled_ = false;
      published_ = false;
      Schedule();
      return;
    }

    // Fill out the send times.
    sender_->CheckOk(sender_->Send(
        fetcher_->context().data, fetcher_->context().size,
        fetcher_->context().monotonic_event_time,
        fetcher_->context().realtime_event_time, monotonic_remote_transmit_time,
        fetcher_->context().queue_index, fetcher_->context().source_boot_uuid));

    // And simulate message_bridge's offset recovery.
    client_status_->SampleFilter(client_index_, monotonic_remote_transmit_time,
                                 sender_->monotonic_sent_time(),
                                 fetcher_->context().source_boot_uuid);

    client_connection_->mutate_received_packets(
        client_connection_->received_packets() + 1);

    if (timestamp_logger_) {
      flatbuffers::FlatBufferBuilder fbb;
      fbb.ForceDefaults(true);
      // Reset the filter every time the UUID changes.  There's probably a more
      // clever way to do this, but that means a better concept of rebooting.
      if (!server_status_->BootUUID(destination_node_index_).has_value() ||
          (server_status_->BootUUID(destination_node_index_).value() !=
           send_node_factory_->boot_uuid())) {
        server_status_->ResetFilter(destination_node_index_);
        server_status_->SetBootUUID(destination_node_index_,
                                    send_node_factory_->boot_uuid());
      }

      flatbuffers::Offset<flatbuffers::Vector<uint8_t>> boot_uuid_offset =
          send_node_factory_->boot_uuid().PackVector(&fbb);

      RemoteMessage::Builder message_header_builder(fbb);

      message_header_builder.add_channel_index(channel_index_);

      // Swap the remote and sent metrics.  They are from the sender's
      // perspective, not the receiver's perspective.
      message_header_builder.add_monotonic_remote_time(
          fetcher_->context().monotonic_event_time.time_since_epoch().count());
      message_header_builder.add_realtime_remote_time(
          fetcher_->context().realtime_event_time.time_since_epoch().count());
      message_header_builder.add_remote_queue_index(
          fetcher_->context().queue_index);
      message_header_builder.add_monotonic_remote_transmit_time(
          monotonic_remote_transmit_time.time_since_epoch().count());
      message_header_builder.add_monotonic_sent_time(
          sender_->monotonic_sent_time().time_since_epoch().count());
      message_header_builder.add_realtime_sent_time(
          sender_->realtime_sent_time().time_since_epoch().count());
      message_header_builder.add_queue_index(sender_->sent_queue_index());
      message_header_builder.add_boot_uuid(boot_uuid_offset);

      fbb.Finish(message_header_builder.Finish());

      remote_timestamps_.emplace_back(
          FlatbufferDetachedBuffer<RemoteMessage>(fbb.Release()),
          fetch_node_factory_->monotonic_now() +
              send_node_factory_->network_delay());
      ScheduleTimestamp();
    }

    sent_ = true;
    reliable_scheduled_ = false;
    published_ = true;
    Schedule();
  }

  // Schedules sending the next timestamp in remote_timestamps_ if there is one.
  void ScheduleTimestamp() {
    if (remote_timestamps_.empty()) {
      timestamp_timer_->Disable();
      return;
    }

    if (scheduled_time_ !=
        remote_timestamps_.front().monotonic_timestamp_time) {
      timestamp_timer_->Schedule(
          remote_timestamps_.front().monotonic_timestamp_time);
      scheduled_time_ = remote_timestamps_.front().monotonic_timestamp_time;
      return;
    } else {
      scheduled_time_ = monotonic_clock::min_time;
    }
  }

  // Sends the next timestamp in remote_timestamps_.
  void SendTimestamp() {
    CHECK(!remote_timestamps_.empty());

    // Send out all timestamps at the currently scheduled time.
    while (remote_timestamps_.front().monotonic_timestamp_time ==
           scheduled_time_) {
      if (server_connection_->state() == State::CONNECTED) {
        timestamp_logger_->CheckOk(timestamp_logger_->Send(
            std::move(remote_timestamps_.front().remote_message)));
      }
      remote_timestamps_.pop_front();
      if (remote_timestamps_.empty()) {
        break;
      }
    }

    ScheduleTimestamp();
  }

  // Converts from time on the sending node to time on the receiving node.
  monotonic_clock::time_point DeliveredTime(
      const monotonic_clock::time_point transmit_time) const {
    const distributed_clock::time_point distributed_sent_time =
        fetch_node_factory_->ToDistributedClock(transmit_time);

    const logger::BootTimestamp t = send_node_factory_->FromDistributedClock(
        distributed_sent_time + send_node_factory_->network_delay());
    CHECK_EQ(t.boot, send_node_factory_->boot_count());
    return t.time;
  }

  const Channel *channel_;
  const Connection *connection_;

  // Factories used for time conversion.
  aos::NodeEventLoopFactory *fetch_node_factory_;
  aos::NodeEventLoopFactory *send_node_factory_;

  // Event loop which fetching and sending timestamps are scheduled on.
  aos::EventLoop *fetch_event_loop_ = nullptr;
  // Event loop which sending is scheduled on.
  aos::EventLoop *send_event_loop_ = nullptr;
  // Timer used to send.
  aos::TimerHandler *timer_ = nullptr;
  // Timer used to send timestamps out.
  aos::TimerHandler *timestamp_timer_ = nullptr;
  // Time that the timer is scheduled for.  Used to track if it needs to be
  // rescheduled.
  monotonic_clock::time_point scheduled_time_ = monotonic_clock::min_time;

  // Fetcher used to receive messages.
  std::unique_ptr<aos::RawFetcher> fetcher_;
  // Sender to send them back out.
  std::unique_ptr<aos::RawSender> sender_;

  MessageBridgeServerStatus *server_status_ = nullptr;
  const size_t destination_node_index_;
  // True if we have sent the message in the fetcher.
  bool sent_ = false;
  bool published_ = false;
  bool reliable_scheduled_ = false;

  ServerConnection *server_connection_ = nullptr;
  int server_index_ = -1;
  MessageBridgeClientStatus *client_status_ = nullptr;
  int client_index_ = -1;
  ClientConnection *client_connection_ = nullptr;

  size_t channel_index_;
  aos::Sender<RemoteMessage> *timestamp_logger_ = nullptr;

  struct TransmitTime {
    monotonic_clock::time_point monotonic_sent_time;
    uint32_t sent_queue_index;
    monotonic_clock::time_point transmit_time;
  };

  // Stores tthe time the message was handed to the kernel to be published on
  // the remote node over the network for all forwarded relevant messages.
  std::vector<TransmitTime> monotonic_remote_transmit_times_;

  struct Timestamp {
    Timestamp(FlatbufferDetachedBuffer<RemoteMessage> new_remote_message,
              monotonic_clock::time_point new_monotonic_timestamp_time)
        : remote_message(std::move(new_remote_message)),
          monotonic_timestamp_time(new_monotonic_timestamp_time) {}
    FlatbufferDetachedBuffer<RemoteMessage> remote_message;
    monotonic_clock::time_point monotonic_timestamp_time;
  };

  std::deque<Timestamp> remote_timestamps_;

  bool delivery_time_is_logged_;

  bool forwarding_disabled_ = false;
};

SimulatedMessageBridge::SimulatedMessageBridge(
    SimulatedEventLoopFactory *simulated_event_loop_factory) {
  CHECK(
      configuration::MultiNode(simulated_event_loop_factory->configuration()));

  // Pre-build up event loops for every node.  They are pretty cheap anyways.
  for (const Node *node : simulated_event_loop_factory->nodes()) {
    NodeEventLoopFactory *node_factory =
        simulated_event_loop_factory->GetNodeEventLoopFactory(node);
    auto it = event_loop_map_.emplace(node, node_factory);
    CHECK(it.second);

    node_factory->OnStartup(
        [this, simulated_event_loop_factory, node_state = &it.first->second]() {
          node_state->MakeEventLoop();
          const size_t my_node_index = configuration::GetNodeIndex(
              simulated_event_loop_factory->configuration(),
              node_state->event_loop->node());

          size_t node_index = 0;
          for (const std::optional<MessageBridgeServerStatus::NodeState>
                   &connection : node_state->server_status_->nodes()) {
            if (connection.has_value()) {
              node_state->server_status_->ResetFilter(node_index);
            }
            ++node_index;
          }

          for (const ClientConnection *client_connections :
               *node_state->client_status->mutable_client_statistics()
                    ->connections()) {
            const Node *client_node = configuration::GetNode(
                simulated_event_loop_factory->configuration(),
                client_connections->node()->name()->string_view());

            auto client_event_loop = event_loop_map_.find(client_node);
            client_event_loop->second.SetBootUUID(
                my_node_index, node_state->event_loop->boot_uuid());
          }
        });

    node_factory->OnShutdown([node_state = &it.first->second]() {
      node_state->SetEventLoop(nullptr);
    });
  }

  for (const Channel *channel :
       *simulated_event_loop_factory->configuration()->channels()) {
    if (!channel->has_destination_nodes()) {
      continue;
    }

    // Find the sending node.
    const Node *source_node =
        configuration::GetNode(simulated_event_loop_factory->configuration(),
                               channel->source_node()->string_view());
    auto source_event_loop = event_loop_map_.find(source_node);
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

      const size_t destination_node_index = configuration::GetNodeIndex(
          simulated_event_loop_factory->configuration(), destination_node);

      const bool delivery_time_is_logged =
          configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                              source_node);

      delayers->v.emplace_back(std::make_unique<RawMessageDelayer>(
          channel, connection,
          simulated_event_loop_factory->GetNodeEventLoopFactory(source_node),
          simulated_event_loop_factory->GetNodeEventLoopFactory(
              destination_node),
          destination_node_index, delivery_time_is_logged));

      source_event_loop->second.AddSourceDelayer(delayers->v.back().get());
      destination_event_loop->second.AddDestinationDelayer(
          delayers->v.back().get());
    }

    const Channel *const timestamp_channel = configuration::GetChannel(
        simulated_event_loop_factory->configuration(), "/aos",
        Timestamp::GetFullyQualifiedName(), "message_bridge", source_node);

    if (channel == timestamp_channel) {
      source_event_loop->second.SetSendData(
          [source_event_loop, captured_delayers = delayers.get()](
              uint32_t sent_queue_index,
              monotonic_clock::time_point monotonic_sent_time) {
            for (std::unique_ptr<RawMessageDelayer> &delayer :
                 captured_delayers->v) {
              delayer->QueueTransmitTimestamp(
                  sent_queue_index, monotonic_sent_time,
                  source_event_loop->second.event_loop->monotonic_now());
              delayer->Schedule();
            }
          });
    } else {
      source_event_loop->second.AddDelayerWatcher(channel, delayers.get());
    }
    delayers_list_.emplace_back(std::move(delayers));
  }
}

SimulatedMessageBridge::~SimulatedMessageBridge() {}

void SimulatedMessageBridge::DisableForwarding(const Channel *channel) {
  for (std::unique_ptr<DelayersVector> &delayers : delayers_list_) {
    if (delayers->v.size() > 0) {
      if (delayers->v[0]->channel() == channel) {
        delayers->disable_forwarding = true;
        for (std::unique_ptr<RawMessageDelayer> &delayer : delayers->v) {
          delayer->set_forwarding_disabled(true);
        }
      }
    }
  }
}

void SimulatedMessageBridge::Disconnect(const Node *source,
                                        const Node *destination) {
  SetState(source, destination, message_bridge::State::DISCONNECTED);
}

void SimulatedMessageBridge::Connect(const Node *source,
                                     const Node *destination) {
  SetState(source, destination, message_bridge::State::CONNECTED);
}
void SimulatedMessageBridge::SetState(const Node *source,
                                      const Node *destination,
                                      message_bridge::State state) {
  auto source_state = event_loop_map_.find(source);
  CHECK(source_state != event_loop_map_.end());
  source_state->second.SetServerState(destination, state);

  auto destination_state = event_loop_map_.find(destination);
  CHECK(destination_state != event_loop_map_.end());
  destination_state->second.SetClientState(source, state);
}

void SimulatedMessageBridge::DisableStatistics(DestroySenders destroy_senders) {
  for (std::pair<const Node *const, State> &state : event_loop_map_) {
    state.second.DisableStatistics(destroy_senders);
  }
}

void SimulatedMessageBridge::DisableStatistics(const Node *node,
                                               DestroySenders destroy_senders) {
  auto it = event_loop_map_.find(node);
  CHECK(it != event_loop_map_.end());
  it->second.DisableStatistics(destroy_senders);
}

void SimulatedMessageBridge::EnableStatistics() {
  for (std::pair<const Node *const, State> &state : event_loop_map_) {
    state.second.EnableStatistics();
  }
}

void SimulatedMessageBridge::EnableStatistics(const Node *node) {
  auto it = event_loop_map_.find(node);
  CHECK(it != event_loop_map_.end());
  it->second.EnableStatistics();
}

void SimulatedMessageBridge::State::MakeEventLoop() {
  // Message bridge isn't the thing that should be catching sent-too-fast,
  // and may need to be able to forward too-fast messages replayed from old
  // logfiles.
  SetEventLoop(node_factory_->MakeEventLoop(
      "message_bridge", {NodeEventLoopFactory::CheckSentTooFast::kNo,
                         NodeEventLoopFactory::ExclusiveSenders::kNo,
                         {}}));
}

void SimulatedMessageBridge::State::SetEventLoop(
    std::unique_ptr<aos::EventLoop> loop) {
  if (!loop) {
    timestamp_loggers = ChannelTimestampSender(nullptr);
    server_status_.reset();
    client_status.reset();
    for (RawMessageDelayer *source_delayer : source_delayers_) {
      source_delayer->SetFetchEventLoop(nullptr, nullptr, nullptr);
    }
    for (RawMessageDelayer *destination_delayer : destination_delayers_) {
      destination_delayer->SetSendEventLoop(nullptr, nullptr);
    }
    event_loop = std::move(loop);
    return;
  } else {
    CHECK(!event_loop);
  }
  event_loop = std::move(loop);

  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  for (std::pair<const Channel *, DelayersVector *> &watcher :
       delayer_watchers_) {
    // Don't register watchers if we know we aren't forwarding.
    if (watcher.second->disable_forwarding) continue;
    event_loop->MakeRawNoArgWatcher(
        watcher.first,
        [this, captured_delayers = watcher.second](const Context &context) {
          // We might get told after registering, so don't forward at that point
          // too.
          for (std::unique_ptr<RawMessageDelayer> &delayer :
               captured_delayers->v) {
            delayer->MessageWatcherCallback(context.queue_index,
                                            context.monotonic_event_time,
                                            event_loop->monotonic_now());
          }
        });
  }

  timestamp_loggers = ChannelTimestampSender(event_loop.get());
  server_status_ =
      std::make_unique<MessageBridgeServerStatus>(event_loop.get());
  if (disable_statistics_) {
    server_status_->DisableStatistics(destroy_senders_ == DestroySenders::kYes);
  }

  {
    size_t node_index = 0;
    for (const std::optional<MessageBridgeServerStatus::NodeState> &connection :
         server_status_->nodes()) {
      if (connection.has_value()) {
        if (boot_uuids_[node_index] != UUID::Zero()) {
          switch (server_state_[node_index]) {
            case message_bridge::State::DISCONNECTED:
              server_status_->Disconnect(node_index);
              break;
            case message_bridge::State::CONNECTED:
              server_status_->Connect(node_index, event_loop->monotonic_now());
              break;
          }
        } else {
          server_status_->Disconnect(node_index);
        }
      }
      ++node_index;
    }
  }

  for (size_t i = 0; i < boot_uuids_.size(); ++i) {
    if (boot_uuids_[i] != UUID::Zero()) {
      server_status_->SetBootUUID(i, boot_uuids_[i]);
    }
  }
  if (fn_) {
    server_status_->set_send_data(fn_);
  }
  client_status = std::make_unique<MessageBridgeClientStatus>(event_loop.get());
  if (disable_statistics_) {
    client_status->DisableStatistics(destroy_senders_ == DestroySenders::kYes);
  }

  for (size_t i = 0;
       i < client_status->mutable_client_statistics()->connections()->size();
       ++i) {
    ClientConnection *client_connection =
        client_status->mutable_client_statistics()
            ->mutable_connections()
            ->GetMutableObject(i);
    const Node *client_node = configuration::GetNode(
        node_factory_->configuration(),
        client_connection->node()->name()->string_view());
    const size_t client_node_index = configuration::GetNodeIndex(
        node_factory_->configuration(), client_node);
    if (boot_uuids_[client_node_index] != UUID::Zero()) {
      if (client_connection->state() != client_state_[client_node_index]) {
        switch (client_state_[client_node_index]) {
          case message_bridge::State::DISCONNECTED:
            client_status->Disconnect(i);
            break;
          case message_bridge::State::CONNECTED:
            client_status->Connect(i);
            break;
        }
      }
    } else {
      client_status->Disconnect(i);
    }
  }

  for (const Channel *channel : *event_loop->configuration()->channels()) {
    CHECK(channel->has_source_node());

    // Sent by us.
    if (configuration::ChannelIsSendableOnNode(channel, event_loop->node()) &&
        channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        const bool delivery_time_is_logged =
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(
                connection, event_loop->node());

        const RawMessageDelayer *delayer = nullptr;
        for (const RawMessageDelayer *candidate : source_delayers_) {
          if (candidate->channel() == channel) {
            delayer = candidate;
          }
        }

        // And the timestamps are then logged back by us again.
        if (!delivery_time_is_logged ||
            CHECK_NOTNULL(delayer)->forwarding_disabled()) {
          continue;
        }

        timestamp_loggers.SenderForChannel(channel, connection);
      }
    }
  }

  for (RawMessageDelayer *source_delayer : source_delayers_) {
    source_delayer->SetFetchEventLoop(event_loop.get(), server_status_.get(),
                                      &timestamp_loggers);
  }
  for (RawMessageDelayer *destination_delayer : destination_delayers_) {
    destination_delayer->SetSendEventLoop(event_loop.get(),
                                          client_status.get());
  }
  event_loop->OnRun([this]() {
    for (RawMessageDelayer *destination_delayer : destination_delayers_) {
      if (destination_delayer->time_to_live() == 0) {
        destination_delayer->ScheduleReliable();
      }
    }
    // Note: This exists to work around the fact that some users like to be able
    // to send reliable messages while execution is stopped, creating a
    // situation where the following sequencing can occur:
    // 1) <While stopped> Send a reliable message on Node A (to be forwarded to
    //    Node B).
    // 2) Node B starts up.
    // 3) Anywhere from 0 to N seconds later, Node A starts up.
    //
    // In this case, we need the reliable message to make it to Node B, but it
    // also shouldn't make it to Node B until Node A has started up.
    //
    // Ideally, if the user were to wait for the Node B OnRun callbacks to send
    // the message, then that would trigger the watchers in the delayers.
    // However, we so far have continued to support Sending while stopped....
    for (RawMessageDelayer *source_delayer : source_delayers_) {
      if (source_delayer->time_to_live() == 0) {
        source_delayer->ScheduleReliable();
      }
    }
  });
}

void SimulatedMessageBridge::State::SetSendData(
    std::function<void(uint32_t, monotonic_clock::time_point)> fn) {
  CHECK(!fn_);
  fn_ = std::move(fn);
  if (server_status_) {
    server_status_->set_send_data(fn_);
  }
}

void SimulatedMessageBridge::State::SetBootUUID(size_t node_index,
                                                const UUID &boot_uuid) {
  boot_uuids_[node_index] = boot_uuid;
  const Node *node = node_factory_->configuration()->nodes()->Get(node_index);
  if (server_status_) {
    ServerConnection *connection = server_status_->FindServerConnection(node);
    if (connection) {
      if (boot_uuid == UUID::Zero()) {
        server_status_->Disconnect(node_index);
        server_status_->ResetFilter(node_index);
      } else {
        switch (server_state_[node_index]) {
          case message_bridge::State::DISCONNECTED:
            server_status_->Disconnect(node_index);
            break;
          case message_bridge::State::CONNECTED:
            server_status_->Connect(node_index, event_loop->monotonic_now());
            break;
        }
        server_status_->ResetFilter(node_index);
        server_status_->SetBootUUID(node_index, boot_uuid);
      }
    }
  }
  if (client_status) {
    const int client_index =
        client_status->FindClientIndex(node->name()->string_view());
    client_status->SampleReset(client_index);
    if (boot_uuid == UUID::Zero()) {
      client_status->Disconnect(client_index);
    } else {
      switch (client_state_[node_index]) {
        case message_bridge::State::CONNECTED:
          client_status->Connect(client_index);
          break;
        case message_bridge::State::DISCONNECTED:
          client_status->Disconnect(client_index);
          break;
      }
    }
  }
}

void SimulatedMessageBridge::State::SetServerState(
    const Node *destination, message_bridge::State state) {
  const size_t node_index =
      configuration::GetNodeIndex(node_factory_->configuration(), destination);
  server_state_[node_index] = state;
  if (server_status_) {
    ServerConnection *connection =
        server_status_->FindServerConnection(destination);
    if (connection == nullptr) return;

    if (state == connection->state()) {
      return;
    }
    switch (state) {
      case message_bridge::State::DISCONNECTED:
        server_status_->Disconnect(node_index);
        break;
      case message_bridge::State::CONNECTED:
        server_status_->Connect(node_index, event_loop->monotonic_now());
        for (RawMessageDelayer *delayer : source_delayers_) {
          if (delayer->SendingTo(destination)) {
            delayer->Connect();
          }
        }
        break;
    }
  }
}

void SimulatedMessageBridge::State::SetClientState(
    const Node *source, message_bridge::State state) {
  const size_t node_index =
      configuration::GetNodeIndex(node_factory_->configuration(), source);
  client_state_[node_index] = state;
  if (client_status) {
    const int client_index =
        client_status->FindClientIndex(source->name()->string_view());
    ClientConnection *connection = client_status->GetClientConnection(source);

    // TODO(austin): Are there cases where we want to dedup 2 CONNECTED
    // calls?
    if (connection->state() != state) {
      switch (state) {
        case message_bridge::State::CONNECTED:
          client_status->Connect(client_index);
          break;
        case message_bridge::State::DISCONNECTED:
          client_status->Disconnect(client_index);
          break;
      }
    }
  }
}

}  // namespace aos::message_bridge
