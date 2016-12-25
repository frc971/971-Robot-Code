#include "aos/common/network/socket.h"

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <chrono>

#include "aos/common/byteorder.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace network {

namespace chrono = ::std::chrono;

int Socket::Connect(NetworkPort port, const char *address, int type) {
  last_ret_ = 0;
  if ((socket_ = socket(AF_INET, type, 0)) < 0) {
    PLOG(ERROR, "failed to create socket");
    return last_ret_ = 1;
  }

  memset(&addr_, 0, sizeof(addr_));
  addr_.in.sin_family = AF_INET;
  addr_.in.sin_port = hton<uint16_t>(static_cast<uint16_t>(port));
#ifndef __VXWORKS__
  const int failure_return = 0;
#else
  const int failure_return = -1;
#endif
  if (inet_aton(address, &addr_.in.sin_addr) == failure_return) {
    PLOG(ERROR, "invalid IP address '%s'", address);
    return last_ret_ = -1;
  }

  return last_ret_ = 0;
}

Socket::Socket() : socket_(-1), last_ret_(2) {}

Socket::~Socket() {
  close(socket_);
}

void Socket::Reset() {
  if (socket_ != -1) {
    close(socket_);
    socket_ = -1;
  }
  last_ret_ = 0;
}

int Socket::Receive(void *buf, int length) {
  const int ret = recv(socket_, static_cast<char *>(buf), length, 0);
  last_ret_ = (ret == -1) ? -1 : 0;
  return ret;
}

int Socket::Receive(void *buf, int length, chrono::microseconds timeout) {
  timeval timeout_timeval;
  timeout_timeval.tv_sec = chrono::duration_cast<chrono::seconds>(timeout).count();
  timeout_timeval.tv_usec =
      chrono::duration_cast<chrono::microseconds>(
          timeout - chrono::seconds(timeout_timeval.tv_sec))
          .count();
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(socket_, &fds);
  switch (select(FD_SETSIZE, &fds, NULL, NULL, &timeout_timeval)) {
    case 1:
      return Receive(buf, length);
    case 0:
      return last_ret_ = 0;
    default:
      if (errno == EINTR) {
        return last_ret_ = 0;
      }
      PLOG(FATAL, "select(FD_SETSIZE, %p, NULL, NULL, %p) failed",
          &fds, &timeout_timeval);
  }
}

int Socket::Send(const void *buf, int length) {
  const int ret = write(socket_,
                        static_cast<const char *>(buf), length);
  last_ret_ = (ret == -1) ? -1 : 0;
  return ret;
}

}  // namespace network
}  // namespace aos
