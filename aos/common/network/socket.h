#ifndef AOS_COMMON_NETWORK_SOCKET_H_
#define AOS_COMMON_NETWORK_SOCKET_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "aos/common/time.h"
#include "aos/common/network_port.h"

namespace aos {
namespace network {

class Socket {
 public:
  int LastStatus() const { return last_ret_; }

  int Send(const void *buf, int length);

  // buf is where to put the data and length is the maximum amount of data to
  // put in for all overloads.
  // All overloads return how many bytes were received or -1 for error. 0 is a
  // valid return value for all overloads.
  // No timeout.
  int Receive(void *buf, int length);
  // timeout is relative
  int Receive(void *buf, int length, time::Time timeout);

 protected:
  int Connect(NetworkPort port, const char *address, int type = SOCK_DGRAM);
  Socket();
  ~Socket();

  // Resets socket_ and last_ret_.
  void Reset();

  union {
    sockaddr_in in;
    sockaddr addr;
  } addr_; // filled in by Connect

  int socket_;
  int last_ret_;
};

}  // namespace network
}  // namespace aos

#endif  // AOS_COMMON_NETWORK_SOCKET_H_
