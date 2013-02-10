#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <errno.h>

#include <map>

#include "aos/aos_core.h"
#include "aos/common/Configuration.h"
#include "aos/common/byteorder.h"

struct log_buffer {
  log_crio_message msg;
  size_t used;

  log_buffer() {
    Clear();
  }
  void Clear() {
    used = 0;
  }
  // returns whether msg is now complete
  bool ReceiveFrom(int sock) {
    const ssize_t ret = recv(sock, reinterpret_cast<uint8_t *>(&msg) + used,
                             sizeof(msg) - used, 0);
    if (ret == -1) {
      LOG(ERROR, "recv(%d, %p, %d) failed because of %d: %s\n",
          sock, reinterpret_cast<uint8_t *>(&msg) + used, sizeof(msg) - used,
          errno, strerror(errno));
      return false;
    } else {
      used += ret;
      if (used > sizeof(msg)) {
        LOG(WARNING, "used(%zd) is > sizeof(msg)(%zd)\n", used, sizeof(msg));
      }
      return used >= sizeof(msg);
    }
  }
};

int main() {
  aos::InitNRT();

  const int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1) {
    LOG(ERROR, "creating TCP socket failed because of %d: %s\n",
        errno, strerror(errno));
    return EXIT_FAILURE;
  }
  union {
    sockaddr_in in;
    sockaddr addr;
  } bind_sockaddr;
  memset(&bind_sockaddr, 0, sizeof(bind_sockaddr));
  bind_sockaddr.in.sin_family = AF_INET;
  bind_sockaddr.in.sin_port = htons(static_cast<uint16_t>(aos::NetworkPort::kLogs));
  bind_sockaddr.in.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, &bind_sockaddr.addr, sizeof(bind_sockaddr.addr)) == -1) {
    LOG(ERROR, "bind(%d, %p) failed because of %d: %s\n", sock, &bind_sockaddr.addr,
        errno, strerror(errno));
    return EXIT_FAILURE;
  }
  if (listen(sock, 10) == -1) {
    LOG(ERROR, "listen(%d) failed because of %d: %s\n", sock,
        errno, strerror(errno));
    return EXIT_FAILURE;
  }
  const int flags = fcntl(sock, F_GETFL, 0);
  if (flags == -1) {
    LOG(ERROR, "fcntl(%d, F_GETFL, 0) failed because of %d: %s\n", sock,
        errno, strerror(errno));
    return EXIT_FAILURE;
  }
  if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
    LOG(ERROR, "fcntl(%d, F_SETFL, %x) failed because of %d: %s\n",
        sock, flags | O_NONBLOCK, errno, strerror(errno));
    return EXIT_FAILURE;
  }

  std::map<int, log_buffer> socks;
  fd_set read_fds;
  while (true) {
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);
    for (auto it = socks.begin(); it != socks.end(); ++it) {
      FD_SET(it->first, &read_fds);
    }
    switch (select(FD_SETSIZE, &read_fds, NULL /*write*/, NULL /*err*/,
                   NULL /*timeout*/)) {
      case -1:
        LOG(ERROR, "select(FD_SETSIZE, %p, NULL, NULL, NULL) failed "
            "because of %d: %s\n", &read_fds, errno, strerror(errno));
        continue;
      case 0:
        LOG(ERROR, "select with NULL timeout timed out...\n");
        continue;
    }

    if (FD_ISSET(sock, &read_fds)) {
      const int new_sock = accept4(sock, NULL, NULL, SOCK_NONBLOCK);
      if (new_sock == -1) {
        LOG(ERROR, "accept4(%d, NULL, NULL, SOCK_NONBLOCK(=%d)) failed "
            "because of %d: %s\n",
            sock, SOCK_NONBLOCK, errno, strerror(errno));
      } else {
        socks[new_sock]; // creates using value's default constructor
      }
    }

    for (auto it = socks.begin(); it != socks.end(); ++it) {
      if (FD_ISSET(it->first, &read_fds)) {
        if (it->second.ReceiveFrom(it->first)) {
          it->second.msg.identifier = -1;
          it->second.msg.time = aos::ntoh(it->second.msg.time);
          log_crio_message_send(it->second.msg);
          it->second.Clear();
        }
      }
    }
  }

  aos::Cleanup();
}
