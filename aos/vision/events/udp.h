#ifndef AOS_VISION_EVENTS_UDP_H_
#define AOS_VISION_EVENTS_UDP_H_

#include <arpa/inet.h>
#include <math.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "aos/common/macros.h"
#include "aos/common/scoped_fd.h"

namespace aos {
namespace events {

// Simple wrapper around a transmitting UDP socket.
//
// LOG(FATAL)s for all errors, including from Send.
class TXUdpSocket {
 public:
  TXUdpSocket(const std::string &ip_addr, int port);

  // Returns the number of bytes actually sent.
  int Send(const char *data, int size);

 private:
  ScopedFD fd_;

  DISALLOW_COPY_AND_ASSIGN(TXUdpSocket);
};

// Simple wrapper around a receiving UDP socket.
//
// LOG(FATAL)s for all errors, including from Recv.
class RXUdpSocket {
 public:
  RXUdpSocket(int port);

  // Returns the number of bytes received.
  int Recv(void *data, int size);

 private:
  ScopedFD fd_;

  DISALLOW_COPY_AND_ASSIGN(RXUdpSocket);
};

}  // namespace events
}  // namespace aos

#endif  // AOS_VISION_EVENTS_UDP_H_
