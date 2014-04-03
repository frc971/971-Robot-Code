#ifndef AOS_COMMON_NETWORK_SEND_SOCKET_H_
#define AOS_COMMON_NETWORK_SEND_SOCKET_H_

#include "aos/common/network/socket.h"

#include "aos/linux_code/configuration.h"
#include "aos/common/network_port.h"
#include "aos/common/util/inet_addr.h"

namespace aos {
namespace network {

class SendSocket : public Socket {
 public:
  // Connect must be called before use.
  SendSocket() {}
  // Calls Connect automatically.
  SendSocket(NetworkPort port, ::aos::NetworkAddress address) {
    Connect(port,
            ::aos::util::MakeIPAddress(::aos::configuration::GetOwnIPAddress(),
                                       address));
  }
  int Connect(NetworkPort port, const char *robot_ip, int type = SOCK_DGRAM);
};

}  // namespace network
}  // namespace aos

#endif  // AOS_COMMON_NETWORK_SEND_SOCKET_H_
