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

  const struct sockaddr_in *const interface_address;
};

void *FDCopyThread(void *to_copy_in) {
  FDsToCopy *to_copy = static_cast<FDsToCopy *>(to_copy_in);

  char buffer[32768];
  ssize_t position = 0;
  while (true) {
    assert(position >= 0);
    assert(position <= static_cast<ssize_t>(sizeof(buffer)));
    if (position != sizeof(buffer)) {
      ssize_t read_bytes;
      bool good_data = true;
      if (to_copy->interface_address != NULL) {
        char control_buffer[0x100];
        struct msghdr header;
        memset(static_cast<void *>(&header), 0, sizeof(header));
        header.msg_control = control_buffer;
        header.msg_controllen = sizeof(control_buffer);
        struct iovec iovecs[1];
        iovecs[0].iov_base = buffer + position;
        iovecs[0].iov_len = position - sizeof(buffer);
        header.msg_iov = iovecs;
        header.msg_iovlen = sizeof(iovecs) / sizeof(iovecs[0]);
        read_bytes = recvmsg(to_copy->input, &header, 0);
        if (read_bytes != -1) {
          for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&header);
               cmsg != NULL;
               cmsg = CMSG_NXTHDR(&header, cmsg)) {
            if (cmsg->cmsg_level == IPPROTO_IP &&
                cmsg->cmsg_type == IP_PKTINFO) {
              struct in_pktinfo *pktinfo =
                  reinterpret_cast<struct in_pktinfo *>(CMSG_DATA(cmsg));
              good_data = pktinfo->ipi_spec_dst.s_addr ==
                  to_copy->interface_address->sin_addr.s_addr;
            }
          }
        }
      } else {
        read_bytes = read(to_copy->input,
                          buffer + position, position - sizeof(buffer));
      }
      if (read_bytes == -1) {
        if (errno != EINTR) {
          LOG(FATAL, "read(%d, %p, %zd) failed with %d: %s\n",
              to_copy->input, buffer + position, position - sizeof(buffer),
              errno, strerror(errno));
        }
      } else if (read_bytes == 0 && to_copy->interface_address == NULL) {
        // read(2) says that this means EOF
        return NULL;
      }
      if (good_data) {
        position += read_bytes;
      }
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

  int input, output;
  if (argc > 1) {
    output = open(argv[1], O_APPEND | O_CREAT | O_WRONLY | O_TRUNC, 0666);
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
    input = -1;
    fprintf(stderr, "Not taking any input.\n");
  } else {
    output = STDOUT_FILENO;
    fprintf(stderr, "Writing output to stdout.\n");
    input = STDIN_FILENO;
    fprintf(stderr, "Reading stdin.\n");
  }

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
  if (setsockopt(from_crio, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on)) == -1) {
    LOG(FATAL, "IPROTO_IP::IP_PKTINFO=%d(%d) failed with %d: %s\n",
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

  pthread_t input_thread, output_thread;

  if (input != -1) {
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
    FDsToCopy input_fds{input, to_crio, NULL};
    if (pthread_create(&input_thread, NULL, FDCopyThread, &input_fds) == -1) {
      LOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed with %d: %s\n",
          &input_thread, FDCopyThread, &input_fds, errno, strerror(errno));
    }
  }

  fprintf(stderr, "Using cRIO IP %s.\n",
          configuration::GetIPAddress(configuration::NetworkDevice::kCRIO));

  if (inet_aton(
          configuration::GetIPAddress(configuration::NetworkDevice::kSelf),
          &address.in.sin_addr) == 0) {
    LOG(FATAL, "inet_aton(%s, %p) failed with %d: %s\n",
        configuration::GetIPAddress(configuration::NetworkDevice::kSelf),
        &address.in.sin_addr, errno, strerror(errno));
  }
  FDsToCopy output_fds{from_crio, output, &address.in};
  if (pthread_create(&output_thread, NULL, FDCopyThread, &output_fds) == -1) {
    LOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed with %d: %s\n",
        &output_thread, FDCopyThread, &output_fds, errno, strerror(errno));
  }

  // input_thread will finish when stdin gets an EOF
  if (pthread_join((input == -1) ? output_thread : input_thread, NULL) == -1) {
    LOG(FATAL, "pthread_join(a_thread, NULL) failed with %d: %s\n",
        errno, strerror(errno));
  }
  exit(EXIT_SUCCESS);
}

}  // namespace
}  // namespace aos

int main(int argc, char **argv) {
  return ::aos::NetconsoleMain(argc, argv);
}
