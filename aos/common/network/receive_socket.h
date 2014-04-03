#ifndef AOS_COMMON_NETWORK_RECEIVE_SOCKET_H_
#define AOS_COMMON_NETWORK_RECEIVE_SOCKET_H_

#include "aos/common/network/socket.h"

namespace aos {
namespace network {

class ReceiveSocket : public Socket {
 public:
  explicit ReceiveSocket(NetworkPort port) { Connect(port); }
  int Connect(NetworkPort port);
};

}  // namespace network
}  // namespace aos

#endif  // AOS_COMMON_NETWORK_RECEIVE_SOCKET_H_
