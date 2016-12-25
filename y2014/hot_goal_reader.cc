#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/byteorder.h"

#include "y2014/queues/hot_goal.q.h"

int main() {
  ::aos::InitNRT();

  uint64_t left_count, right_count;
  ::y2014::hot_goal.FetchLatest();
  if (::y2014::hot_goal.get()) {
    LOG_STRUCT(DEBUG, "starting with", *::y2014::hot_goal);
    left_count = ::y2014::hot_goal->left_count;
    right_count = ::y2014::hot_goal->left_count;
  } else {
    LOG(DEBUG, "no starting message\n");
    left_count = right_count = 0;
  }

  int my_socket = -1;
  while (true) {
    if (my_socket == -1) {
      my_socket = socket(AF_INET, SOCK_STREAM, 0);
      if (my_socket == -1) {
        PLOG(WARNING, "socket(AF_INET, SOCK_STREAM, 0) failed");
        continue;
      } else {
        LOG(INFO, "opened socket (is %d)\n", my_socket);
        sockaddr_in address, *sockaddr_pointer;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = ::aos::hton<uint16_t>(1180);
        sockaddr *address_pointer;
        sockaddr_pointer = &address;
        memcpy(&address_pointer, &sockaddr_pointer, sizeof(void *));
        if (bind(my_socket, address_pointer, sizeof(address)) == -1) {
          PLOG(WARNING, "bind(%d, %p, %zu) failed",
               my_socket, &address, sizeof(address));
          close(my_socket);
          my_socket = -1;
          continue;
        }

        if (listen(my_socket, 1) == -1) {
          PLOG(WARNING, "listen(%d, 1) failed", my_socket);
          close(my_socket);
          my_socket = -1;
          continue;
        }
      }
    }

    int connection = accept4(my_socket, nullptr, nullptr, SOCK_NONBLOCK);
    if (connection == -1) {
      PLOG(WARNING, "accept(%d, nullptr, nullptr) failed", my_socket);
      continue;
    }
    LOG(INFO, "accepted (is %d)\n", connection);

    while (connection != -1) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(connection, &fds);
      struct timeval timeout_timeval;
      timeout_timeval.tv_sec = 1;
      timeout_timeval.tv_usec = 0;
      switch (
          select(connection + 1, &fds, nullptr, nullptr, &timeout_timeval)) {
        case 1: {
          uint8_t data;
          ssize_t read_bytes = read(connection, &data, sizeof(data));
          if (read_bytes != sizeof(data)) {
            LOG(WARNING, "read %zd bytes instead of %zd\n", read_bytes,
                sizeof(data));
            break;
          }
          if (data & 0x01) ++right_count;
          if (data & 0x02) ++left_count;
          auto message = ::y2014::hot_goal.MakeMessage();
          message->left_count = left_count;
          message->right_count = right_count;
          LOG_STRUCT(DEBUG, "sending", *message);
          message.Send();
        } break;
        case 0:
          LOG(WARNING, "read on %d timed out\n", connection);
          close(connection);
          connection = -1;
          break;
        default:
          PLOG(FATAL,
               "select(%d, %p, nullptr, nullptr, %p) failed",
               connection + 1, &fds, &timeout_timeval);
      }
    }
  }

  LOG(FATAL, "finished???\n");
}
