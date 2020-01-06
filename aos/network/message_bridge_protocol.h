#ifndef AOS_NETWORK_MESSAGE_BRIDGE_PROTOCOL_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_PROTOCOL_H_

namespace aos {
namespace message_bridge {

// The protocol between the message_bridge_client and server is pretty simple.
// The overarching design philosophy is that the server sends data to the
// client, and the client (optionally) sends timestamps back.
//
// 1) A connection is established by the client sending the server a Connect
//    flatbuffer on stream 0.
// 2) The server then replies with the data, as it is available, on streams 2 +
//    channel_id in the Connect message.
// 3) The client (optionally) replies on stream 1 with MessageHeader flatbuffers
//    with the timestamps that the messages were received.
//
// Most of the complexity from there is handling multiple clients and servers
// and persuading SCTP to do what we want.

// Number of streams reserved for control messages.
constexpr size_t kControlStreams() { return 2; }
// The stream on which Connect messages are sent.
constexpr size_t kConnectStream() { return 0; }
// The stream on which timestamp replies are sent.
constexpr size_t kTimestampStream() { return 1; }

}  // namespace message_bridge
}  // namespace aos

#endif  // AOS_NETWORK_MESSAGE_BRIDGE_PROTOCOL_H_
