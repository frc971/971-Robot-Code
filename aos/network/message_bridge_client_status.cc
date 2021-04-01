#include "aos/network/message_bridge_client_status.h"

#include <chrono>
#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"

namespace aos {
namespace message_bridge {
namespace {
namespace chrono = std::chrono;

FlatbufferDetachedBuffer<ClientStatistics> MakeClientStatistics(
    const std::vector<std::string_view> &source_node_names) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  std::vector<flatbuffers::Offset<ClientConnection>> connection_offsets;
  for (const std::string_view node_name : source_node_names) {
    flatbuffers::Offset<flatbuffers::String> node_name_offset =
        fbb.CreateString(node_name);

    Node::Builder node_builder(fbb);
    node_builder.add_name(node_name_offset);
    flatbuffers::Offset<Node> node_offset = node_builder.Finish();

    ClientConnection::Builder connection_builder(fbb);
    connection_builder.add_node(node_offset);
    connection_builder.add_state(State::DISCONNECTED);
    // TODO(austin): Track dropped packets.
    connection_builder.add_received_packets(0);
    connection_builder.add_duplicate_packets(0);
    connection_builder.add_monotonic_offset(0);
    connection_builder.add_partial_deliveries(0);
    connection_offsets.emplace_back(connection_builder.Finish());
  }
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ClientConnection>>>
      connections_offset = fbb.CreateVector(connection_offsets);

  ClientStatistics::Builder client_statistics_builder(fbb);
  client_statistics_builder.add_connections(connections_offset);
  fbb.Finish(client_statistics_builder.Finish());

  return fbb.Release();
}

}  // namespace

MessageBridgeClientStatus::MessageBridgeClientStatus(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<ClientStatistics>("/aos")),
      source_node_names_(configuration::SourceNodeNames(
          event_loop->configuration(), event_loop->node())),
      statistics_(MakeClientStatistics(source_node_names_)) {
  client_connection_offsets_.reserve(
      statistics_.message().connections()->size());
  filters_.resize(statistics_.message().connections()->size());

  statistics_timer_ = event_loop_->AddTimer([this]() { SendStatistics(); });
  statistics_timer_->set_name("statistics");
  event_loop_->OnRun([this]() {
    if (send_) {
      statistics_timer_->Setup(
          event_loop_->monotonic_now() + chrono::milliseconds(100),
          chrono::milliseconds(100));
    }
  });
}

void MessageBridgeClientStatus::SendStatistics() {
  if (!send_) {
    return;
  }
  // Copy from statistics_ and drop monotonic_offset if it isn't populated yet.
  // There doesn't exist a good way to drop fields otherwise.
  aos::Sender<ClientStatistics>::Builder builder = sender_.MakeBuilder();
  client_connection_offsets_.clear();

  for (const ClientConnection *connection :
       *statistics_.message().connections()) {
    flatbuffers::Offset<flatbuffers::String> node_name_offset =
        builder.fbb()->CreateString(connection->node()->name()->string_view());
    Node::Builder node_builder = builder.MakeBuilder<Node>();
    node_builder.add_name(node_name_offset);
    flatbuffers::Offset<Node> node_offset = node_builder.Finish();

    ClientConnection::Builder client_connection_builder =
        builder.MakeBuilder<ClientConnection>();

    client_connection_builder.add_node(node_offset);
    client_connection_builder.add_state(connection->state());
    client_connection_builder.add_received_packets(
        connection->received_packets());
    if (connection->duplicate_packets() != 0) {
      client_connection_builder.add_duplicate_packets(
          connection->duplicate_packets());
    }
    client_connection_builder.add_partial_deliveries(
        connection->partial_deliveries());

    // Strip out the monotonic offset if it isn't populated.
    TimestampFilter *filter = &filters_[client_connection_offsets_.size()];
    if (filter->has_sample()) {
      client_connection_builder.add_monotonic_offset(
          (chrono::duration_cast<chrono::nanoseconds>(
               chrono::duration<double>(filter->offset())) +
           filter->base_offset())
              .count());
    }

    client_connection_offsets_.emplace_back(client_connection_builder.Finish());
  }

  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ClientConnection>>>
      client_connections_offset =
          builder.fbb()->CreateVector(client_connection_offsets_);

  ClientStatistics::Builder client_statistics_builder =
      builder.MakeBuilder<ClientStatistics>();
  client_statistics_builder.add_connections(client_connections_offset);

  builder.Send(client_statistics_builder.Finish());
}

int MessageBridgeClientStatus::FindClientIndex(std::string_view node_name) {
  for (size_t i = 0; i < statistics_.message().connections()->size(); ++i) {
    const ClientConnection *client_connection =
        statistics_.message().connections()->Get(i);
    if (client_connection->node()->name()->string_view() == node_name) {
      return i;
    }
  }

  LOG(FATAL) << "Unknown client " << node_name;
}

ClientConnection *MessageBridgeClientStatus::GetClientConnection(
    int client_index) {
  return statistics_.mutable_message()->mutable_connections()->GetMutableObject(
      client_index);
}

ClientConnection *MessageBridgeClientStatus::GetClientConnection(
    const Node *node) {
  return GetClientConnection(FindClientIndex(node->name()->string_view()));
}

void MessageBridgeClientStatus::SampleFilter(
    int client_index,
    const aos::monotonic_clock::time_point monotonic_sent_time,
    const aos::monotonic_clock::time_point monotonic_delivered_time) {
  TimestampFilter *filter = &filters_[client_index];

  const std::chrono::nanoseconds offset =
      monotonic_delivered_time - monotonic_sent_time;

  // If this is our first observation, use that to seed the base offset.  That
  // gets us in the ballpark.
  if (!filter->has_sample()) {
    filter->set_base_offset(offset);
  }

  // We can now measure the latency!
  filter->Sample(monotonic_delivered_time, offset);
}

void MessageBridgeClientStatus::DisableStatistics() {
  statistics_timer_->Disable();
  // TODO(austin): Re-arm when re-enabled.
  send_ = false;
}

}  // namespace message_bridge
}  // namespace aos
