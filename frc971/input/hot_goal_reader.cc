#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/byteorder.h"

#include "frc971/queues/hot_goal.q.h"

int main() {
  ::aos::InitNRT();

  uint64_t left_count, right_count;
  ::frc971::hot_goal.FetchLatest();
  if (::frc971::hot_goal.get()) {
    LOG_STRUCT(DEBUG, "starting with", *::frc971::hot_goal);
    left_count = ::frc971::hot_goal->left_count;
    right_count = ::frc971::hot_goal->left_count;
  } else {
    LOG(DEBUG, "no starting message\n");
    left_count = right_count = 0;
  }

  int my_socket = -1;
  while (true) {
    if (my_socket == -1) {
      my_socket = socket(AF_INET, SOCK_STREAM, 0);
      if (my_socket == -1) {
        LOG(WARNING, "socket(AF_INET, SOCK_STREAM, 0) failed with %d: %s\n",
            errno, strerror(errno));
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
          LOG(WARNING, "bind(%d, %p, %zu) failed with %d: %s\n",
              my_socket, &address, sizeof(address), errno, strerror(errno));
          close(my_socket);
          my_socket = -1;
          continue;
        }

        if (listen(my_socket, 1) == -1) {
          LOG(WARNING, "listen(%d, 1) failed with %d: %s\n",
              my_socket, errno, strerror(errno));
          close(my_socket);
          my_socket = -1;
          continue;
        }
      }
    }

    int connection = accept4(my_socket, nullptr, nullptr, SOCK_NONBLOCK);
    if (connection == -1) {
      LOG(WARNING, "accept(%d, nullptr, nullptr) failed with %d: %s\n",
          my_socket, errno, strerror(errno));
      continue;
    }
    LOG(INFO, "accepted (is %d)\n", connection);

    while (connection != -1) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(connection, &fds);
      struct timeval timeout_timeval =
          ::aos::time::Time::InSeconds(1).ToTimeval();
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
          auto message = ::frc971::hot_goal.MakeMessage();
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
          LOG(FATAL,
              "select(%d, %p, nullptr, nullptr, %p) failed with %d: %s\n",
              connection + 1, &fds, &timeout_timeval, errno, strerror(errno));
      }
    }
  }

  LOG(FATAL, "finished???\n");
}
