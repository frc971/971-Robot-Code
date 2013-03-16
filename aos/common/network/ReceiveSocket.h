#ifndef AOS_NETWORK_RECEIVE_SOCKET_H_
#define AOS_NETWORK_RECEIVE_SOCKET_H_

#include "Socket.h"

namespace aos {

class ReceiveSocket : public Socket {
 public:
  inline ReceiveSocket(NetworkPort port) { Connect(port); }
  int Connect(NetworkPort port);
};

} // namespace aos

#endif
