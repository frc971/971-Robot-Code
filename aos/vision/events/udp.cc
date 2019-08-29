#include "aos/vision/events/udp.h"

#include <string.h>

#include "glog/logging.h"

namespace aos {
namespace events {

TXUdpSocket::TXUdpSocket(const std::string &ip_addr, int port)
    : fd_(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) {
  PCHECK(fd_.get() != -1);
  sockaddr_in destination_in;
  memset(&destination_in, 0, sizeof(destination_in));
  destination_in.sin_family = AF_INET;
  destination_in.sin_port = htons(port);
  CHECK(inet_aton(ip_addr.c_str(), &destination_in.sin_addr) != 0)
      << ": invalid IP address " << ip_addr;

  PCHECK(connect(fd_.get(), reinterpret_cast<sockaddr *>(&destination_in),
                 sizeof(destination_in)) == 0);
}

int TXUdpSocket::Send(const char *data, int size) {
  // Don't fail on send. If no one is connected that is fine.
  return send(fd_.get(), data, size, 0);
}

int RXUdpSocket::SocketBindListenOnPort(int port) {
  int fd;
  PCHECK((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1);
  sockaddr_in bind_address;
  memset(&bind_address, 0, sizeof(bind_address));

  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);

  PCHECK(bind(fd, reinterpret_cast<sockaddr *>(&bind_address),
              sizeof(bind_address)) == 0);
  return fd;
}

RXUdpSocket::RXUdpSocket(int port) : fd_(SocketBindListenOnPort(port)) {}

int RXUdpSocket::Recv(void *data, int size) {
  int result;
  PCHECK((result = recv(fd_.get(), static_cast<char *>(data), size, 0)) != -1);
  return result;
}

}  // namespace events
}  // namespace aos
