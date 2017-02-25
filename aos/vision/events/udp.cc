#include "aos/vision/events/udp.h"

#include <string.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace events {

TXUdpSocket::TXUdpSocket(const std::string &ip_addr, int port)
    : fd_(PCHECK(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))) {
  sockaddr_in destination_in;
  memset(&destination_in, 0, sizeof(destination_in));
  destination_in.sin_family = AF_INET;
  destination_in.sin_port = htons(port);
  if (inet_aton(ip_addr.c_str(), &destination_in.sin_addr) == 0) {
    LOG(FATAL, "invalid IP address %s\n", ip_addr.c_str());
  }

  PCHECK(connect(fd_.get(), reinterpret_cast<sockaddr *>(&destination_in),
                 sizeof(destination_in)));
}

int TXUdpSocket::Send(const char *data, int size) {
  // Don't fail on send. If no one is connected that is fine.
  return send(fd_.get(), data, size, 0);
}

RXUdpSocket::RXUdpSocket(int port)
    : fd_(PCHECK(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))) {
  sockaddr_in bind_address;
  memset(&bind_address, 0, sizeof(bind_address));

  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);

  PCHECK(bind(fd_.get(), reinterpret_cast<sockaddr *>(&bind_address),
              sizeof(bind_address)));
}

int RXUdpSocket::Recv(void *data, int size) {
  return PCHECK(recv(fd_.get(), static_cast<char *>(data), size, 0));
}

}  // namespace events
}  // namespace aos
