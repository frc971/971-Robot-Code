#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "aos/byteorder.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "y2014/queues/hot_goal_generated.h"

int main() {
  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop shm_event_loop(&config.message());

  ::aos::Sender<::y2014::HotGoal> hot_goal_sender =
      shm_event_loop.MakeSender<::y2014::HotGoal>("/");

  uint64_t left_count = 0, right_count = 0;
  int my_socket = -1;
  while (true) {
    if (my_socket == -1) {
      my_socket = socket(AF_INET, SOCK_STREAM, 0);
      if (my_socket == -1) {
        AOS_PLOG(WARNING, "socket(AF_INET, SOCK_STREAM, 0) failed");
        continue;
      } else {
        AOS_LOG(INFO, "opened socket (is %d)\n", my_socket);
        sockaddr_in address, *sockaddr_pointer;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = ::aos::hton<uint16_t>(1180);
        sockaddr *address_pointer;
        sockaddr_pointer = &address;
        memcpy(&address_pointer, &sockaddr_pointer, sizeof(void *));
        if (bind(my_socket, address_pointer, sizeof(address)) == -1) {
          AOS_PLOG(WARNING, "bind(%d, %p, %zu) failed", my_socket, &address,
                   sizeof(address));
          close(my_socket);
          my_socket = -1;
          continue;
        }

        if (listen(my_socket, 1) == -1) {
          AOS_PLOG(WARNING, "listen(%d, 1) failed", my_socket);
          close(my_socket);
          my_socket = -1;
          continue;
        }
      }
    }

    int connection = accept4(my_socket, nullptr, nullptr, SOCK_NONBLOCK);
    if (connection == -1) {
      AOS_PLOG(WARNING, "accept(%d, nullptr, nullptr) failed", my_socket);
      continue;
    }
    AOS_LOG(INFO, "accepted (is %d)\n", connection);

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
            AOS_LOG(WARNING, "read %zd bytes instead of %zd\n", read_bytes,
                    sizeof(data));
            break;
          }
          if (data & 0x01) ++right_count;
          if (data & 0x02) ++left_count;
          auto builder = hot_goal_sender.MakeBuilder();
          y2014::HotGoal::Builder hot_goal_builder =
              builder.MakeBuilder<y2014::HotGoal>();
          hot_goal_builder.add_left_count(left_count);
          hot_goal_builder.add_right_count(right_count);
          builder.Send(hot_goal_builder.Finish());
        } break;
        case 0:
          AOS_LOG(WARNING, "read on %d timed out\n", connection);
          close(connection);
          connection = -1;
          break;
        default:
          AOS_PLOG(FATAL, "select(%d, %p, nullptr, nullptr, %p) failed",
                   connection + 1, &fds, &timeout_timeval);
      }
    }
  }

  AOS_LOG(FATAL, "finished???\n");
}
