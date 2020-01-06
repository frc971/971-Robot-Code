#ifndef AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_

#include <string_view>

#include "aos/events/event_loop.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/sctp_client.h"
#include "aos/network/sctp_lib.h"

namespace aos {
namespace message_bridge {

// See message_bridge_protocol.h for more details about the protocol.

// This class encapsulates all the state required to connect to a server and
// transmit messages.
class SctpClientConnection {
 public:
  SctpClientConnection(aos::ShmEventLoop *const event_loop,
                       std::string_view remote_name, const Node *my_node,
                       std::string_view local_host,
                       std::vector<std::unique_ptr<aos::RawSender>> *channels,
                       ClientConnection *connection);

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
  std::vector<std::unique_ptr<aos::RawSender>> *channels_;
  // Stream number -> channel lookup.
  std::vector<int> stream_to_channel_;
  // Bitmask signaling if we should be replying back with delivery times.
  std::vector<bool> stream_reply_with_timestamp_;

  // Timer which fires to handle reconnections.
  aos::TimerHandler *connect_timer_;

  // ClientConnection statistics message to modify.  This will be published
  // periodicially.
  ClientConnection *connection_;

  // id of the server once known.  This is only valid if connection_ says
  // connected.
  sctp_assoc_t remote_assoc_id_ = 0;
};

// This encapsulates the state required to talk to *all* the servers from this
// node.
class MessageBridgeClient {
 public:
  MessageBridgeClient(aos::ShmEventLoop *event_loop);

  ~MessageBridgeClient() {}

 private:
  // Sends out the statistics that are continually updated by the
  // SctpClientConnections.
  void SendStatistics() { sender_.Send(statistics_); }

  // Event loop to schedule everything on.
  aos::ShmEventLoop *event_loop_;
  // Sender to publish statistics on.
  aos::Sender<ClientStatistics> sender_;
  aos::TimerHandler *statistics_timer_;

  // Nodes to receive data from.
  const std::vector<std::string_view> source_node_names_;

  // Data to publish.
  FlatbufferDetachedBuffer<ClientStatistics> statistics_;

  // Channels to send data over.
  std::vector<std::unique_ptr<aos::RawSender>> channels_;

  // List of connections.  These correspond to the nodes in source_node_names_
  std::vector<std::unique_ptr<SctpClientConnection>> connections_;
};

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_CLIENT_LIB_H_
