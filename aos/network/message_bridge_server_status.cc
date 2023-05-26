#include "aos/network/message_bridge_server_status.h"

#include <chrono>
#include <functional>

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/network/timestamp_generated.h"

namespace aos {
namespace message_bridge {
namespace {

namespace chrono = std::chrono;

std::vector<const Channel *> ChannelsForNodePair(
    const Configuration *configuration, const aos::Node *source_node,
    const aos::Node *destination_node) {
  std::vector<const Channel *> result;
  for (size_t channel_index = 0;
       channel_index < configuration->channels()->size(); ++channel_index) {
    const aos::Channel *channel = configuration->channels()->Get(channel_index);
    if (configuration::ChannelIsForwardedFromNode(channel, source_node) &&
        configuration::ChannelIsReadableOnNode(channel, destination_node)) {
      result.push_back(channel);
    }
  }
  return result;
}

// Builds up the "empty" server statistics message to be pointed to by all the
// connections, updated at runtime, and periodically sent.
FlatbufferDetachedBuffer<ServerStatistics> MakeServerStatistics(
    const std::vector<std::string_view> &source_node_names,
    const Configuration *configuration) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  std::vector<flatbuffers::Offset<ServerConnection>> connection_offsets;
  for (const std::string_view node_name : source_node_names) {
    flatbuffers::Offset<Node> node_offset = RecursiveCopyFlatBuffer(
        configuration::GetNode(configuration, node_name), &fbb);
    ServerConnection::Builder connection_builder(fbb);
    connection_builder.add_node(node_offset);
    connection_builder.add_state(State::DISCONNECTED);
    connection_builder.add_dropped_packets(0);
    connection_builder.add_sent_packets(0);
    connection_builder.add_monotonic_offset(0);
    connection_builder.add_partial_deliveries(0);
    connection_builder.add_connected_since_time(
        monotonic_clock::min_time.time_since_epoch().count());
    connection_builder.add_connection_count(0);
    connection_builder.add_invalid_connection_count(0);
    connection_offsets.emplace_back(connection_builder.Finish());
  }
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ServerConnection>>>
      connections_offset = fbb.CreateVector(connection_offsets);

  ServerStatistics::Builder server_statistics_builder(fbb);
  server_statistics_builder.add_connections(connections_offset);
  fbb.Finish(server_statistics_builder.Finish());

  return fbb.Release();
}

// Finds the statistics for the provided node name.
ServerConnection *FindServerConnection(ServerStatistics *statistics,
                                       std::string_view node_name) {
  ServerConnection *matching_server_connection = nullptr;
  for (size_t i = 0; i < statistics->mutable_connections()->size(); ++i) {
    ServerConnection *server_connection =
        statistics->mutable_connections()->GetMutableObject(i);
    if (server_connection->node()->name()->string_view() == node_name) {
      matching_server_connection = server_connection;
      break;
    }
  }

  CHECK(matching_server_connection != nullptr) << ": Unknown client";

  return matching_server_connection;
}

}  // namespace

MessageBridgeServerStatus::MessageBridgeServerStatus(
    aos::EventLoop *event_loop, std::function<void(const Context &)> send_data)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<ServerStatistics>("/aos")),
      statistics_(MakeServerStatistics(
          configuration::DestinationNodeNames(event_loop_->configuration(),
                                              event_loop_->node()),
          event_loop->configuration())),
      client_statistics_fetcher_(
          event_loop_->MakeFetcher<ClientStatistics>("/aos")),
      timestamp_sender_(event_loop_->MakeSender<Timestamp>("/aos")),
      send_data_(send_data) {
  server_connection_offsets_.reserve(
      statistics_.message().connections()->size());
  client_offsets_.reserve(statistics_.message().connections()->size());

  nodes_.resize(event_loop->configuration()->nodes()->size());

  // Seed up all the per-node connection state.
  // We are making the assumption here that every connection is bidirectional
  // (data is being sent both ways).  This is pretty safe because we are
  // forwarding timestamps between nodes.
  for (std::string_view destination_node_name :
       configuration::DestinationNodeNames(event_loop->configuration(),
                                           event_loop->node())) {
    const Node *destination_node = configuration::GetNode(
        event_loop->configuration(), destination_node_name);

    const int node_index = configuration::GetNodeIndex(
        event_loop->configuration(), destination_node);

    // Now find the timestamp channel forwarded from the other node.
    const Channel *const other_timestamp_channel =
        configuration::GetChannel(event_loop_->configuration(), "/aos",
                                  Timestamp::GetFullyQualifiedName(),
                                  event_loop_->name(), destination_node);

    ServerConnection *const server_connection =
        FindServerConnection(destination_node->name()->string_view());
    std::map<const Channel *, ServerChannelStatisticsT> channel_statistics;
    for (const Channel *channel :
         ChannelsForNodePair(event_loop_->configuration(), event_loop_->node(),
                             destination_node)) {
      ServerChannelStatisticsT initial_statistics;
      initial_statistics.channel_index =
          configuration::ChannelIndex(event_loop_->configuration(), channel);
      initial_statistics.sent_packets = 0;
      initial_statistics.dropped_packets = 0;
      channel_statistics[channel] = initial_statistics;
    }

    nodes_[node_index] =
        NodeState{.server_connection = server_connection,
                  .channel_statistics = channel_statistics,
                  .channel_offsets_buffer =
                      std::vector<flatbuffers::Offset<ServerChannelStatistics>>(
                          channel_statistics.size()),
                  .timestamp_fetcher = event_loop_->MakeFetcher<Timestamp>(
                      other_timestamp_channel->name()->string_view()),
                  .filter = {},
                  .boot_uuid = std::nullopt};
  }

  statistics_timer_ = event_loop_->AddTimer([this]() { Tick(); });
  statistics_timer_->set_name(absl::StrCat(
      event_loop_->node()->name()->string_view(), "_server_statistics"));
  event_loop_->OnRun([this]() {
    if (send_) {
      statistics_timer_->Schedule(event_loop_->monotonic_now() + kPingPeriod,
                                  kPingPeriod);
    }
  });
}

ServerConnection *MessageBridgeServerStatus::FindServerConnection(
    std::string_view node_name) {
  return message_bridge::FindServerConnection(statistics_.mutable_message(),
                                              node_name);
}

ServerConnection *MessageBridgeServerStatus::FindServerConnection(
    const Node *node) {
  return FindServerConnection(node->name()->string_view());
}

void MessageBridgeServerStatus::AddSentPacket(int node_index,
                                              const aos::Channel *channel) {
  CHECK(nodes_[node_index].has_value());
  NodeState &node = nodes_[node_index].value();
  ServerConnection *connection = node.server_connection;
  connection->mutate_sent_packets(connection->sent_packets() + 1);
  node.channel_statistics[channel].sent_packets++;
}
void MessageBridgeServerStatus::AddDroppedPacket(int node_index,
                                                 const aos::Channel *channel) {
  CHECK(nodes_[node_index].has_value());
  NodeState &node = nodes_[node_index].value();
  ServerConnection *connection = node.server_connection;
  connection->mutate_dropped_packets(connection->dropped_packets() + 1);
  node.channel_statistics[channel].dropped_packets++;
}

void MessageBridgeServerStatus::SetBootUUID(int node_index,
                                            const UUID &boot_uuid) {
  nodes_[node_index].value().boot_uuid = boot_uuid;
  SendStatistics();
  last_statistics_send_time_ = event_loop_->monotonic_now();
}

void MessageBridgeServerStatus::ClearBootUUID(int node_index) {
  nodes_[node_index].value().boot_uuid.reset();
  SendStatistics();
  last_statistics_send_time_ = event_loop_->monotonic_now();
}

void MessageBridgeServerStatus::ResetFilter(int node_index) {
  nodes_[node_index].value().filter.Reset();
  nodes_[node_index].value().server_connection->mutate_monotonic_offset(0);
}

void MessageBridgeServerStatus::Connect(
    int node_index, monotonic_clock::time_point monotonic_now) {
  ServerConnection *message = nodes_[node_index].value().server_connection;
  message->mutate_state(State::CONNECTED);
  // Only count connections if the timestamp changes.  This deduplicates
  // multiple channel connections at the same point in time.
  if (message->connected_since_time() !=
      monotonic_now.time_since_epoch().count()) {
    message->mutate_connection_count(message->connection_count() + 1);
    message->mutate_connected_since_time(
        monotonic_now.time_since_epoch().count());
  }
}

void MessageBridgeServerStatus::Disconnect(int node_index) {
  ServerConnection *message = nodes_[node_index].value().server_connection;
  message->mutate_state(State::DISCONNECTED);
  message->mutate_connected_since_time(
      aos::monotonic_clock::min_time.time_since_epoch().count());
}

void MessageBridgeServerStatus::SendStatistics() {
  if (!send_) return;
  aos::Sender<ServerStatistics>::Builder builder = sender_.MakeBuilder();

  server_connection_offsets_.clear();

  // Copy the statistics over, but only add monotonic_offset if it is valid.
  for (const ServerConnection *connection :
       *statistics_.message().connections()) {
    const int node_index =
        configuration::GetNodeIndex(event_loop_->configuration(),
                                    connection->node()->name()->string_view());
    CHECK(nodes_[node_index].has_value());

    flatbuffers::Offset<flatbuffers::String> node_name_offset =
        builder.fbb()->CreateString(connection->node()->name()->string_view());
    Node::Builder node_builder = builder.MakeBuilder<Node>();
    node_builder.add_name(node_name_offset);
    flatbuffers::Offset<Node> node_offset = node_builder.Finish();

    flatbuffers::Offset<flatbuffers::String> boot_uuid_offset;
    if (connection->state() == State::CONNECTED &&
        nodes_[node_index].value().boot_uuid.has_value()) {
      boot_uuid_offset =
          nodes_[node_index].value().boot_uuid.value().PackString(
              builder.fbb());
    }

    {
      size_t index = 0;
      CHECK_EQ(nodes_[node_index].value().channel_offsets_buffer.size(),
               nodes_[node_index].value().channel_statistics.size());
      for (const auto &channel :
           nodes_[node_index].value().channel_statistics) {
        nodes_[node_index].value().channel_offsets_buffer.at(index) =
            ServerChannelStatistics::Pack(*builder.fbb(), &channel.second);
        index++;
      }
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<ServerChannelStatistics>>>
        channels_offset = builder.fbb()->CreateVector(
            nodes_[node_index].value().channel_offsets_buffer);

    ServerConnection::Builder server_connection_builder =
        builder.MakeBuilder<ServerConnection>();
    server_connection_builder.add_node(node_offset);
    server_connection_builder.add_state(connection->state());
    server_connection_builder.add_dropped_packets(
        connection->dropped_packets());
    server_connection_builder.add_sent_packets(connection->sent_packets());
    server_connection_builder.add_partial_deliveries(
        PartialDeliveries(node_index));
    server_connection_builder.add_channels(channels_offset);

    if (connection->connected_since_time() !=
        monotonic_clock::min_time.time_since_epoch().count()) {
      server_connection_builder.add_connected_since_time(
          connection->connected_since_time());
    }

    if (connection->connection_count() != 0) {
      server_connection_builder.add_connection_count(
          connection->connection_count());
    }

    if (connection->invalid_connection_count() != 0) {
      server_connection_builder.add_invalid_connection_count(
          connection->invalid_connection_count());
    }

    // TODO(austin): If it gets stale, drop it too.
    if (!nodes_[node_index].value().filter.MissingSamples()) {
      server_connection_builder.add_monotonic_offset(
          connection->monotonic_offset());
    }

    if (!boot_uuid_offset.IsNull()) {
      server_connection_builder.add_boot_uuid(boot_uuid_offset);
    }

    server_connection_offsets_.emplace_back(server_connection_builder.Finish());
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ServerConnection>>>
      server_connections_offset =
          builder.fbb()->CreateVector(server_connection_offsets_);

  ServerStatistics::Builder server_statistics_builder =
      builder.MakeBuilder<ServerStatistics>();
  server_statistics_builder.add_connections(server_connections_offset);
  server_statistics_builder.add_timestamp_send_failures(
      timestamp_failure_counter_.failures());
  server_statistics_builder.add_invalid_connection_count(
      invalid_connection_count_);

  builder.CheckOk(builder.Send(server_statistics_builder.Finish()));
}

void MessageBridgeServerStatus::Tick() {
  // Send statistics every kStatisticsPeriod. Use the context so we don't get
  // caught up with the wakeup delay and jitter.
  if (event_loop_->context().monotonic_event_time >=
      last_statistics_send_time_ + kStatisticsPeriod) {
    SendStatistics();
    last_statistics_send_time_ = event_loop_->context().monotonic_event_time;
  }

  // The message_bridge_client application measures and filters the offsets from
  // all messages it receives.  It then sends this on in the ClientStatistics
  // message.  Collect that up and forward it back over the Timestamp message so
  // we have guarenteed traffic on the other node for timestamping.  This also
  // moves the offsets back across the network so both directions can be
  // observed.
  client_statistics_fetcher_.Fetch();

  // Build up the timestamp message.  Do it here so that we don't have invalid
  // data in it.
  FlatbufferFixedAllocatorArray<Timestamp, 1000> timestamp_copy;
  flatbuffers::FlatBufferBuilder *fbb = timestamp_copy.fbb();

  if (client_statistics_fetcher_.get()) {
    // Build up the list of client offsets.
    client_offsets_.clear();

    // Iterate through the connections this node has made.
    for (const ClientConnection *connection :
         *client_statistics_fetcher_->connections()) {
      const int node_index = configuration::GetNodeIndex(
          event_loop_->configuration(),
          connection->node()->name()->string_view());

      // Filter out the ones which aren't connected.
      // And the ones without monotonic offsets.
      if (connection->state() != State::CONNECTED ||
          !connection->has_monotonic_offset() ||
          client_statistics_fetcher_.context().monotonic_event_time +
                  MessageBridgeServerStatus::kClientStatisticsStaleTimeout <
              event_loop_->context().monotonic_event_time) {
        VLOG(1) << "Disconnected, no offset, or client message too old for "
                << connection->node()->name()->string_view();
        ResetFilter(node_index);
        continue;
      }

      aos::Fetcher<Timestamp> &timestamp_fetcher =
          nodes_[node_index].value().timestamp_fetcher;
      timestamp_fetcher.Fetch();

      // Find the offset computed on their node for this client connection
      // using their timestamp message.
      bool has_their_offset = false;
      chrono::nanoseconds their_offset = chrono::nanoseconds(0);
      if (timestamp_fetcher.get() != nullptr) {
        for (const ClientOffset *client_offset :
             *timestamp_fetcher->offsets()) {
          if (client_offset->node()->name()->string_view() ==
              event_loop_->node()->name()->string_view()) {
            // Make sure it has an offset and the message isn't stale.
            if (client_offset->has_monotonic_offset()) {
              if (timestamp_fetcher.context().monotonic_event_time +
                      MessageBridgeServerStatus::kTimestampStaleTimeout >
                  event_loop_->context().monotonic_event_time) {
                their_offset =
                    chrono::nanoseconds(client_offset->monotonic_offset());
                has_their_offset = true;
              } else {
                ResetFilter(node_index);
                VLOG(1) << "Timestamp old, resetting.";
              }
            }
            break;
          }
        }
      }

      ServerConnection *message = nodes_[node_index].value().server_connection;
      ClippedAverageFilter &filter = nodes_[node_index].value().filter;
      if (has_their_offset && message->state() == State::CONNECTED) {
        // Update the filters.
        if (filter.MissingSamples()) {
          // Update the offset the first time.  This should be representative.
          filter.set_base_offset(
              chrono::nanoseconds(connection->monotonic_offset()));
        }
        // The message_bridge_clients are the ones running the first filter.  So
        // set the values from that and let the averaging filter run from there.
        filter.FwdSet(timestamp_fetcher.context().monotonic_remote_time,
                      chrono::nanoseconds(connection->monotonic_offset()));
        filter.RevSet(client_statistics_fetcher_.context().monotonic_event_time,
                      their_offset);

        // Publish!
        message->mutate_monotonic_offset(-filter.offset().count());
      }

      // Now fill out the Timestamp message with the offset from the client.
      flatbuffers::Offset<flatbuffers::String> node_name_offset =
          fbb->CreateString(connection->node()->name()->string_view());

      Node::Builder node_builder(*fbb);
      node_builder.add_name(node_name_offset);
      flatbuffers::Offset<Node> node_offset = node_builder.Finish();

      ClientOffset::Builder client_offset_builder(*fbb);
      client_offset_builder.add_node(node_offset);
      client_offset_builder.add_monotonic_offset(
          connection->monotonic_offset());
      client_offsets_.emplace_back(client_offset_builder.Finish());
    }
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ClientOffset>>>
        offsets_offset = fbb->CreateVector(client_offsets_);

    Timestamp::Builder builder(*fbb);
    builder.add_offsets(offsets_offset);
    timestamp_copy.Finish(builder.Finish());
  } else {
    // Publish an empty timestamp if we have nothing.
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ClientOffset>>>
        offsets_offset =
            fbb->CreateVector(std::vector<flatbuffers::Offset<ClientOffset>>{});
    Timestamp::Builder builder(*fbb);
    builder.add_offsets(offsets_offset);
    timestamp_copy.Finish(builder.Finish());
  }

  // Send it out over shm, and using that timestamp, then send it out over sctp.
  // This avoid some context switches.
  if (!send_) return;

  const auto err = timestamp_sender_.Send(timestamp_copy);
  timestamp_failure_counter_.Count(err);
  // Reply only if we successfully sent the timestamp
  if (err == RawSender::Error::kOk) {
    Context context;
    context.monotonic_event_time = timestamp_sender_.monotonic_sent_time();
    context.realtime_event_time = timestamp_sender_.realtime_sent_time();
    context.queue_index = timestamp_sender_.sent_queue_index();
    context.size = timestamp_copy.span().size();
    context.source_boot_uuid = event_loop_->boot_uuid();
    context.data = timestamp_copy.span().data();

    // Since we are building up the timestamp to send here, we need to trigger
    // the SendData call ourselves.
    if (send_data_) {
      send_data_(context);
    }
  }
}

void MessageBridgeServerStatus::DisableStatistics(bool destroy_senders) {
  send_ = false;
  statistics_timer_->Disable();
  if (destroy_senders) {
    sender_ = aos::Sender<ServerStatistics>();
    timestamp_sender_ = aos::Sender<Timestamp>();
  }
}

void MessageBridgeServerStatus::EnableStatistics() {
  send_ = true;
  CHECK(sender_.valid());
  CHECK(timestamp_sender_.valid());
  statistics_timer_->Schedule(event_loop_->monotonic_now() + kPingPeriod,
                              kPingPeriod);
}

}  // namespace message_bridge
}  // namespace aos
