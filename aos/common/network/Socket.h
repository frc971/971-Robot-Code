#ifndef AOS_NETWORK_SOCKET_H_
#define AOS_NETWORK_SOCKET_H_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include "aos/common/Configuration.h"

namespace aos {

class Socket {
 public:
  int LastStatus() const { return last_ret_; }

	int Send(const void *buf, int length);
	int Recv(void *buf, int length);
	int Recv(void *buf, int length, long usec); // returns 0 if timed out
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

} // namespace aos

#endif

