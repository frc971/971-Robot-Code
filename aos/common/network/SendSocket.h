#ifndef AOS_NETWORK_SEND_SOCKET_H_
#define AOS_NETWORK_SEND_SOCKET_H_

#include "Socket.h"

namespace aos {

class SendSocket : public Socket {
 public:
  // Connect must be called before use.
  SendSocket() {}
  // Calls Connect automatically.
  SendSocket(NetworkPort port, const char *robot_ip) {
    Connect(port, robot_ip);
  }
  int Connect(NetworkPort port, const char *robot_ip, int type = SOCK_DGRAM);
};

} // namespace aos

#endif

