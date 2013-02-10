#ifndef AOS_NETWORK_RECEIVE_SOCKET_H_
#define AOS_NETWORK_RECEIVE_SOCKET_H_

#include "Socket.h"

namespace aos {

class ReceiveSocket : public Socket {
 public:
	inline ReceiveSocket(NetworkPort port) { Connect(port); }
  int Connect(NetworkPort port);

  inline int Recv(void *buf, int length) { return Socket::Recv(buf, length); }
  inline int Recv(void *buf, int length, long usec) { return Socket::Recv(buf, length, usec); }
};

} // namespace aos

#endif
