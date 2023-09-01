#ifndef AOS_NETWORK_MESSAGE_BRIGE_AUTH_CLIENT_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIGE_AUTH_CLIENT_LIB_H_

#include "aos/events/event_loop.h"
#include "aos/network/message_bridge_auth.grpc.pb.h"
#include "aos/network/sctp_config_generated.h"
#include "aos/network/sctp_config_request_generated.h"
#include "grpcpp/channel.h"

namespace aos::message_bridge::auth {

// See the explanation of underlying the architecture in
// aos/network/message_bridge_auth_server_lib.h.

// This class propagates the SCTP authentication key from the gRPC server to
// message bridge. It listens to requests in the /aos SctpConfigRequest channel
// and sends responses to /aos SctpConfig channel.
class MessageBridgeAuthClient {
 public:
  MessageBridgeAuthClient(EventLoop *const event_loop,
                          std::shared_ptr<grpc::Channel> channel);

  // Gets the key from the gRPC channel and sends it to the /aos SctpConfig
  // channel.
  void SendKey();

 private:
  // Gets the active SCTP key from the authentication server.
  //
  // Returns an empty vector on failure.
  std::vector<uint8_t> GetSctpKey();

  EventLoop *const event_loop_;
  Sender<SctpConfig> sender_;
  TimerHandler *poll_timer_;
  Fetcher<SctpConfigRequest> config_request_fetcher_;
  const std::unique_ptr<SctpConfigService::Stub> client_;
};
}  // namespace aos::message_bridge::auth

#endif  // AOS_NETWORK_MESSAGE_BRIGE_AUTH_CLIENT_LIB_H_
