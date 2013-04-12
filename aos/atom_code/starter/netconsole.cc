#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <assert.h>

#include "aos/common/logging/logging_impl.h"
#include "aos/common/Configuration.h"

namespace aos {
namespace {

struct FDsToCopy {
  const int input;
  const int output;
};

void *FDCopyThread(void *to_copy_in) {
  FDsToCopy *to_copy = static_cast<FDsToCopy *>(to_copy_in);

  char buffer[32768];
  ssize_t position = 0;
  while (true) {
    assert(position >= 0);
    assert(position <= static_cast<ssize_t>(sizeof(buffer)));
    if (position != sizeof(buffer)) {
      ssize_t read_bytes = read(to_copy->input,
                                buffer + position, position - sizeof(buffer));
      if (read_bytes == -1) {
        if (errno != EINTR) {
          LOG(FATAL, "read(%d, %p, %zd) failed with %d: %s\n",
              to_copy->input, buffer + position, position - sizeof(buffer),
              errno, strerror(errno));
        }
      } else if (read_bytes == 0) {
        // read(2) says that this means EOF
        return NULL;
      }
      position += read_bytes;
    }

    assert(position >= 0);
    assert(position <= static_cast<ssize_t>(sizeof(buffer)));
    if (position > 0) {
      ssize_t sent_bytes = write(to_copy->output, buffer, position);
      if (sent_bytes == -1) {
        if (errno != EINTR) {
          LOG(FATAL, "write(%d, %p, %zd) failed with %d: %s\n",
              to_copy->output, buffer, position, errno, strerror(errno));
        }
      } else if (sent_bytes != 0) {
        memmove(buffer, buffer + sent_bytes, position - sent_bytes);
        position -= sent_bytes;
      }
    }
  }
}

int NetconsoleMain(int argc, char **argv) {
  logging::Init();

  int output;
  if (argc > 1) {
    output = open(argv[1], O_APPEND | O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (output == -1) {
      if (errno == EACCES || errno == ELOOP || errno == ENOSPC ||
          errno == ENOTDIR || errno == EROFS || errno == ETXTBSY) {
        fprintf(stderr, "Opening output file '%s' failed because of %s.\n",
                argv[1], strerror(errno));
        exit(EXIT_FAILURE);
      }
      LOG(FATAL, "open('%s', stuff, 0644) failed with %d: %s\n", argv[1],
          errno, strerror(errno));
    }
    fprintf(stderr, "Writing output to '%s'.\n", argv[1]);
  } else {
    output = STDOUT_FILENO;
    fprintf(stderr, "Writing output to stdout.\n");
  }

  const int input = STDIN_FILENO;

  int on = 1;

  int from_crio = socket(AF_INET, SOCK_DGRAM, 0);
  if (from_crio == -1) {
    LOG(FATAL, "socket(AF_INET, SOCK_DGRAM, 0) failed with %d: %s\n",
        errno, strerror(errno));
  }
  if (setsockopt(from_crio, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
    LOG(FATAL, "SOL_SOCKET::SO_REUSEADDR=%d(%d) failed with %d: %s\n",
        on, from_crio, errno, strerror(errno));
  }
  if (setsockopt(from_crio, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) == -1) {
    LOG(FATAL, "SOL_SOCKET::SO_BROADCAST=%d(%d) failed with %d: %s\n",
        on, from_crio, errno, strerror(errno));
  }
  union {
    struct sockaddr_in in;
    struct sockaddr addr;
  } address;
  address.in.sin_family = AF_INET;
  address.in.sin_port = htons(6666);
  address.in.sin_addr.s_addr = INADDR_ANY;
  if (bind(from_crio, &address.addr, sizeof(address)) == -1) {
    LOG(FATAL, "bind(%d, %p, %zu) failed with %d: %s\n",
        from_crio, &address.addr, sizeof(address), errno, strerror(errno));
  }

  int to_crio = socket(AF_INET, SOCK_DGRAM, 0);
  if (to_crio == -1) {
    LOG(FATAL, "socket(AF_INET, SOCK_DGRAM, 0) failed with %d: %s\n",
        errno, strerror(errno));
  }
  if (setsockopt(to_crio, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
    LOG(FATAL, "SOL_SOCKET::SO_REUSEADDR=%d(%d) failed with %d: %s\n",
      on, to_crio, errno, strerror(errno));
  }
  address.in.sin_port = htons(6668);
  if (inet_aton(
          configuration::GetIPAddress(configuration::NetworkDevice::kCRIO),
          &address.in.sin_addr) == 0) {
    LOG(FATAL, "inet_aton(%s, %p) failed with %d: %s\n",
        configuration::GetIPAddress(configuration::NetworkDevice::kCRIO),
        &address.in.sin_addr, errno, strerror(errno));
  }
  if (connect(to_crio, &address.addr, sizeof(address)) == -1) {
    LOG(FATAL, "connect(%d, %p, %zu) failed with %d: %s\n",
        to_crio, &address.addr, sizeof(address), errno, strerror(errno));
  }

  fprintf(stderr, "Using cRIO IP %s.\n",
          configuration::GetIPAddress(configuration::NetworkDevice::kCRIO));

  FDsToCopy output_fds{from_crio, output};
  pthread_t output_thread;
  if (pthread_create(&output_thread, NULL, FDCopyThread, &output_fds) == -1) {
    LOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed with %d: %s\n",
        &output_thread, FDCopyThread, &output_fds, errno, strerror(errno));
  }
  FDsToCopy input_fds{input, to_crio};
  pthread_t input_thread;
  if (pthread_create(&input_thread, NULL, FDCopyThread, &input_fds) == -1) {
    LOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed with %d: %s\n",
        &input_thread, FDCopyThread, &input_fds, errno, strerror(errno));
  }

  // input_thread will finish when stdin gets an EOF
  if (pthread_join(input_thread, NULL) == -1) {
    LOG(FATAL, "pthread_join(input_thread, NULL) failed with %d: %s\n",
        errno, strerror(errno));
  }
  exit(EXIT_SUCCESS);
}

}  // namespace
}  // namespace aos

int main(int argc, char **argv) {
  return ::aos::NetconsoleMain(argc, argv);
}
