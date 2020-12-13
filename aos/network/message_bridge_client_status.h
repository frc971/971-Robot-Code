#ifndef AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_STATUS_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_STATUS_H_

#include <string_view>
#include <vector>

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/timestamp_filter.h"

namespace aos {
namespace message_bridge {

// This class is responsible for publishing the (filtered) client side
// statistics periodically.
class MessageBridgeClientStatus {
 public:
  MessageBridgeClientStatus(aos::EventLoop *event_loop);

  MessageBridgeClientStatus(const MessageBridgeClientStatus &) = delete;
  MessageBridgeClientStatus(MessageBridgeClientStatus &&) = delete;
  MessageBridgeClientStatus &operator=(const MessageBridgeClientStatus &) =
      delete;
  MessageBridgeClientStatus &operator=(MessageBridgeClientStatus &&) = delete;

  // Returns the connection datastructure for the provided node.
  int FindClientIndex(std::string_view node_name);
  ClientConnection *GetClientConnection(int client_index);
  ClientConnection *GetClientConnection(const Node *node);

  // Returns the ClientStatistics message this holds.
  ClientStatistics *mutable_client_statistics() {
    return statistics_.mutable_message();
  }

  // Adds a sample for the provided client index given the sent time (on the
  // remote node) and the delivered time (on this node).
  void SampleFilter(
      int client_index,
      const aos::monotonic_clock::time_point monotonic_sent_time,
      const aos::monotonic_clock::time_point monotonic_delivered_time);

  // Clears out the filter state.
  void SampleReset(int client_index) { filters_[client_index].Reset(); }

  // Disables sending out any statistics messages.
  void DisableStatistics();

 private:
  // Sends out the statistics that are continually updated by the
  // SctpClientConnections.
  void SendStatistics();

  aos::EventLoop *event_loop_;
  aos::TimerHandler *statistics_timer_;

  // Sender to publish statistics on.
  aos::Sender<ClientStatistics> sender_;

  // Nodes to receive data from.
  const std::vector<std::string_view> source_node_names_;
  // Data to publish.
  FlatbufferDetachedBuffer<ClientStatistics> statistics_;
  // Reserved memory for the client connection offsets to reduce heap
  // allocations.
  std::vector<flatbuffers::Offset<ClientConnection>> client_connection_offsets_;

  std::vector<TimestampFilter> filters_;

  // If true, send out the messages.
  bool send_ = true;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_STATUS_H_
