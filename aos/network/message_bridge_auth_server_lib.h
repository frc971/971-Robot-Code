#ifndef AOS_NETWORK_MESSAGE_BRIGE_AUTH_SERVER_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIGE_AUTH_SERVER_LIB_H_

#include <string>

#include "aos/network/message_bridge_auth.grpc.pb.h"
#include "aos/network/message_bridge_auth.pb.h"
#include "grpc/grpc.h"

namespace aos::message_bridge::auth {

// We use two services to share the active SCTP authentication key. For context,
// if SCTP authentication is wanted, then we will need a way to securely
// distribute a shared key across all the nodes. We use gRPC to distribute the
// key.
//
// * message_bridge_auth_server
//
// This service should only run on a single node. It generates an n-bit
// random-key during initialization. The server and client should use a mutal
// authenticated channel. Without mutual authentication, anyone would be able to
// acquire the authentication key which would defeat the purpose.
//
// * message_bridge_auth_client
//
// This service will run on all nodes. It listens  for requests in /aos
// aos.message_bridge.SctpConfigRequest and requests the active key from the
// gRPC server which gets propagated into /aos aos.message_bridge.SctpConfig.
// message_bridge reads this value and sets the authentication key (previous
// change in relation).

// This class implements the SctpConfigService. It securely generates an SCTP
// authentication key and sends it to the clients that request it.
class SctpConfigServer final : public SctpConfigService::Service {
 public:
  SctpConfigServer();

  grpc::Status GetActiveKey(grpc::ServerContext *context,
                            const SctpKeyRequest *_request,
                            SctpKeyResponse *response) override;

 private:
  std::string active_key_;
};

}  // namespace aos::message_bridge::auth

#endif  // AOS_NETWORK_MESSAGE_BRIGE_AUTH_SERVER_LIB_H_
