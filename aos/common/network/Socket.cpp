#include "aos/common/network/Socket.h"

#include <string.h>
#include <errno.h>

#include "aos/common/logging/logging.h"
#include "aos/common/network/SocketLibraries.h"

namespace aos {

int Socket::Connect(NetworkPort port, const char *address, int type) {
  last_ret_ = 0;
  if ((socket_ = socket(AF_INET, type, 0)) < 0) {
    LOG(ERROR, "failed to create socket because of %d: %s\n",
        errno, strerror(errno));
    return last_ret_ = 1;
  }

  memset(&addr_, 0, sizeof(addr_));
  addr_.in.sin_family = AF_INET;
  addr_.in.sin_port = htons(static_cast<uint16_t>(port));
#ifndef __VXWORKS__
  const int failure_return = 0;
#else
  const int failure_return = -1;
#endif
  if (inet_aton(lame_unconst(address), &addr_.in.sin_addr) == failure_return) {
    LOG(ERROR, "Invalid IP address '%s' because of %d: %s\n", address,
        errno, strerror(errno));
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

int Socket::Receive(void *buf, int length, time::Time timeout) {
  timeval timeout_timeval = timeout.ToTimeval();
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
      LOG(FATAL, "select(FD_SETSIZE, %p, NULL, NULL, %p) failed with %d: %s\n",
          &fds, &timeout_timeval, errno, strerror(errno));
  }
}

int Socket::Send(const void *buf, int length) {
  const int ret = write(socket_,
                        lame_unconst(static_cast<const char *>(buf)), length);
  last_ret_ = (ret == -1) ? -1 : 0;
  return ret;
}

}  // namespace aos
