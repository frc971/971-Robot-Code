#ifndef AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_

#include <string_view>

#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_client_status.h"
#include "aos/network/sctp_client.h"
#include "aos/network/sctp_lib.h"

namespace aos {
namespace message_bridge {

// Structure to hold per channel state.
struct SctpClientChannelState {
  // The sender for a channel.
  std::unique_ptr<aos::RawSender> sender;
  // The last queue index of a message sent.  Used for detecting duplicates.
  uint32_t last_queue_index = 0xffffffff;
  // The last timestamp of a message sent.  Used for detecting duplicates.
  monotonic_clock::time_point last_timestamp = monotonic_clock::min_time;
};

// See message_bridge_protocol.h for more details about the protocol.

// This class encapsulates all the state required to connect to a server and
// transmit messages.
class SctpClientConnection {
 public:
  static constexpr std::chrono::seconds kReconnectTimeout{3};
  SctpClientConnection(aos::ShmEventLoop *const event_loop,
                       std::string_view remote_name, const Node *my_node,
                       std::string_view local_host,
                       std::vector<SctpClientChannelState> *channels,
                       int client_index,
                       MessageBridgeClientStatus *client_status,
                       std::string_view config_sha256,
                       std::vector<uint8_t> auth_key);

  ~SctpClientConnection() { event_loop_->epoll()->DeleteFd(client_.fd()); }

 private:
  // Reads a message from the socket.  Could be a notification.
  void MessageReceived();

  // Sends a connection request message.
  void SendConnect();

  // Called when the server connection succeeds.
  void NodeConnected(sctp_assoc_t assoc_id);
  // Called when the server connection disconnects.
  void NodeDisconnected();
  void HandleData(const Message *message);

  // Schedules connect_timer_ for a ways in the future. If one of our messages
  // gets dropped, the server might be waiting for this, so if we don't hear
  // from the server for a while we'll try sending it again.
  void ScheduleConnectTimeout() {
    connect_timer_->Schedule(event_loop_->context().monotonic_event_time +
                             kReconnectTimeout);
  }

  // Event loop to register the server on.
  aos::ShmEventLoop *const event_loop_;

  // Message to send on connect.
  const aos::FlatbufferDetachedBuffer<aos::message_bridge::Connect>
      connect_message_;

  // Starting point for the message reception reply (including timestamps).
  aos::FlatbufferDetachedBuffer<aos::logger::MessageHeader>
      message_reception_reply_;

  // Node we are sending to.
  const aos::Node *const remote_node_;

  // SCTP client.  There is a client per connection so we don't have to deal
  // with association ids nearly as badly.
  SctpClient client_;

  // Channels to send received messages on.
  std::vector<SctpClientChannelState> *channels_;
  // Stream number -> channel lookup.
  std::vector<int> stream_to_channel_;
  // Bitmask signaling if we should be replying back with delivery times.
  std::vector<bool> stream_reply_with_timestamp_;

  // Timer which fires to handle reconnections.
  aos::TimerHandler *connect_timer_;

  // ClientConnection statistics message to modify.  This will be published
  // periodicially.
  MessageBridgeClientStatus *client_status_;
  int client_index_;
  ClientConnection *connection_;
};

// This encapsulates the state required to talk to *all* the servers from this
// node.
class MessageBridgeClient {
 public:
  // When the `auth_key` byte-vector is non-empty, it will be used as the shared
  // key to authenticate every channel (See RFC4895 for more info).
  MessageBridgeClient(aos::ShmEventLoop *event_loop, std::string config_sha256,
                      std::vector<uint8_t> auth_key);

  ~MessageBridgeClient() {}

 private:
  // Event loop to schedule everything on.
  aos::ShmEventLoop *event_loop_;

  MessageBridgeClientStatus client_status_;

  // Channels to send data over.
  std::vector<SctpClientChannelState> channels_;

  // List of connections.  These correspond to the nodes in source_node_names_
  std::vector<std::unique_ptr<SctpClientConnection>> connections_;

  std::string config_sha256_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_
