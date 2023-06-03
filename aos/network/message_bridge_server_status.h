#ifndef AOS_NETWORK_MESSAGE_BRIDGE_SERVER_STATUS_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_SERVER_STATUS_H_

#include <chrono>
#include <functional>
#include <map>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/timestamp_filter.h"
#include "aos/network/timestamp_generated.h"
#include "aos/time/time.h"

namespace aos {
namespace message_bridge {

// This class encapsulates the server side of sending server statistics and
// managing timestamp offsets.
class MessageBridgeServerStatus {
 public:
  // Time after which we consider the client statistics message stale, and reset
  // the filter.
  static constexpr std::chrono::seconds kClientStatisticsStaleTimeout{1};
  // Time after which we consider the timestamp stale, and reset the filter.
  static constexpr std::chrono::milliseconds kTimestampStaleTimeout{1000};

  // Struct containing all of the relevant state for a given node.
  struct NodeState {
    // Mutable status for this node, to be sent out in the ServerStatistics
    // message.
    ServerConnection *server_connection;
    // Mapping of channel index in the Configuration to the statistics for that
    // channel.
    std::map<const aos::Channel *, ServerChannelStatisticsT> channel_statistics;
    // Buffer for the above offsets, because flatbuffers doesn't provide a great
    // API for creating vectors of tables (namely, CreateUninitializedVector
    // doesn't work for tables because it can't handle offsets and
    // CreateVector(len, generator_function) creates an intermediate
    // std::vector).
    std::vector<flatbuffers::Offset<ServerChannelStatistics>>
        channel_offsets_buffer;
    // Fetcher to retrieve timestamps for the connection to the other node,
    // for feeding the timestamp filter.
    aos::Fetcher<Timestamp> timestamp_fetcher;
    // Filter for calculating current time offsets to the other node.
    ClippedAverageFilter filter;
    // Current boot UUID of the other node, if available.
    std::optional<UUID> boot_uuid;
    uint32_t partial_deliveries = 0;
  };

  MessageBridgeServerStatus(
      aos::EventLoop *event_loop,
      std::function<void()> send_data = std::function<void()>());

  MessageBridgeServerStatus(const MessageBridgeServerStatus &) = delete;
  MessageBridgeServerStatus(MessageBridgeServerStatus &&) = delete;
  MessageBridgeServerStatus &operator=(const MessageBridgeServerStatus &) =
      delete;
  MessageBridgeServerStatus &operator=(MessageBridgeServerStatus &&) = delete;

  void set_send_data(std::function<void()> send_data) {
    send_data_ = send_data;
  }

  // Resets the filter and clears the entry from the server statistics.
  void ResetFilter(int node_index);
  // Sets the boot UUID for the provided node.
  void SetBootUUID(int node_index, const UUID &boot_uuid);
  // Clears the boot UUID for the provided node.
  void ClearBootUUID(int node_index);

  void Connect(int node_index, monotonic_clock::time_point monotonic_now);
  void Disconnect(int node_index);

  // Returns the boot UUID for a node, or nullopt if there isn't one.
  const std::optional<UUID> &BootUUID(int node_index) const {
    return nodes_[node_index].value().boot_uuid;
  }

  void AddPartialDeliveries(int node_index, uint32_t partial_deliveries) {
    nodes_[node_index].value().partial_deliveries += partial_deliveries;
  }

  void ResetPartialDeliveries(int node_index) {
    nodes_[node_index].value().partial_deliveries = 0;
  }

  uint32_t PartialDeliveries(int node_index) const {
    return nodes_[node_index].value().partial_deliveries;
  }

  // Track an additional sent/dropped packets on each channel. node_index
  // represents the node being sent to.
  // node_index must be a valid client node.
  void AddSentPacket(int node_index, const aos::Channel *channel);
  void AddDroppedPacket(int node_index, const aos::Channel *channel);
  void AddPacketRetry(int node_index, const aos::Channel *channel);

  // Returns the ServerConnection message which is updated by the server.
  ServerConnection *FindServerConnection(std::string_view node_name);
  ServerConnection *FindServerConnection(const Node *node);

  const std::vector<std::optional<NodeState>> &nodes() { return nodes_; }

  // Disables sending out any statistics messages.
  void DisableStatistics(bool destroy_senders);
  // Enables sending out any statistics messages.
  void EnableStatistics();

  // Increments invalid_connection_count_, marking that we had another bad
  // connection that got rejected.
  void increment_invalid_connection_count() { ++invalid_connection_count_; }

 private:
  static constexpr std::chrono::nanoseconds kStatisticsPeriod =
      std::chrono::seconds(1);
  static constexpr std::chrono::nanoseconds kPingPeriod =
      std::chrono::milliseconds(100);

  // Handle timestamps and statistics.
  void Tick();

  // Sends out the statistics that are continually updated by the
  // ChannelState's.
  void SendStatistics();

  aos::EventLoop *event_loop_;

  // Statistics, timer, and associated sender.
  aos::Sender<ServerStatistics> sender_;
  aos::TimerHandler *statistics_timer_;
  FlatbufferDetachedBuffer<ServerStatistics> statistics_;
  std::vector<flatbuffers::Offset<ServerConnection>> server_connection_offsets_;

  // Fetcher to grab the measured offsets in the client.
  aos::Fetcher<ClientStatistics> client_statistics_fetcher_;

  // ServerConnection to fill out the offsets for from each node.
  std::vector<std::optional<NodeState>> nodes_;

  // Sender for the timestamps that we are forwarding over the network.
  aos::Sender<Timestamp> timestamp_sender_;

  SendFailureCounter timestamp_failure_counter_;

  aos::monotonic_clock::time_point last_statistics_send_time_ =
      aos::monotonic_clock::min_time;

  std::function<void()> send_data_;

  bool send_ = true;

  size_t invalid_connection_count_ = 0u;

  std::vector<flatbuffers::Offset<ClientOffset>> client_offsets_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_SERVER_STATUS_H_
