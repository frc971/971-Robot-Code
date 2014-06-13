#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>

#include "aos/common/logging/logging_impl.h"
#include "aos/common/util/inet_addr.h"
#include "aos/linux_code/configuration.h"
#include "aos/common/network_port.h"
#include "aos/linux_code/init.h"
#include "aos/common/byteorder.h"

namespace aos {
namespace {

struct FDsToCopy {
  const int input;
  const int output;

  const struct sockaddr_in *const interface_address;
  const struct sockaddr_in *const source_address;
};

void *FDCopyThread(void *to_copy_in) {
  FDsToCopy *to_copy = static_cast<FDsToCopy *>(to_copy_in);

  char buffer[32768];
  ssize_t position = 0;
  while (true) {
    CHECK_GE(position, 0);
    CHECK_LE(position, static_cast<ssize_t>(sizeof(buffer)));
    if (position != sizeof(buffer)) {
      ssize_t read_bytes;
      bool good_data = true;
      if (to_copy->interface_address != nullptr ||
          to_copy->source_address != nullptr) {
        char control_buffer[0x100];
        struct msghdr header;
        memset(static_cast<void *>(&header), 0, sizeof(header));
        header.msg_control = control_buffer;
        header.msg_controllen = sizeof(control_buffer);
        struct iovec iovecs[1];
        iovecs[0].iov_base = buffer + position;
        iovecs[0].iov_len = sizeof(buffer) - position;
        header.msg_iov = iovecs;
        header.msg_iovlen = sizeof(iovecs) / sizeof(iovecs[0]);
        struct sockaddr_in sender_address;
        header.msg_name = &sender_address;
        header.msg_namelen = sizeof(sender_address);

        read_bytes = recvmsg(to_copy->input, &header, 0);
        if (read_bytes != -1) {
          if (to_copy->interface_address != nullptr) {
            for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&header);
                 cmsg != NULL;
                 cmsg = CMSG_NXTHDR(&header, cmsg)) {
              if (cmsg->cmsg_level == IPPROTO_IP &&
                  cmsg->cmsg_type == IP_PKTINFO) {
                unsigned char *data = CMSG_DATA(cmsg);
                struct in_pktinfo *pktinfo;
                memcpy(&pktinfo, &data, sizeof(void *));
                if (pktinfo->ipi_spec_dst.s_addr !=
                    to_copy->interface_address->sin_addr.s_addr) {
                  good_data = false;
                }
              }
            }
          }
          if (to_copy->source_address != nullptr) {
            CHECK_GE(header.msg_namelen, sizeof(struct sockaddr_in));
            if (to_copy->source_address->sin_port != hton<uint16_t>(0)) {
              if (sender_address.sin_port !=
                  to_copy->source_address->sin_port) {
                good_data = false;
              }
            }
            if (sender_address.sin_addr.s_addr !=
                to_copy->source_address->sin_addr.s_addr) {
              good_data = false;
            }
          }
        }
      } else {
        read_bytes = read(to_copy->input,
                          buffer + position, sizeof(buffer) - position);
      }
      if (read_bytes == -1) {
        if (errno != EINTR) {
          PLOG(FATAL, "read(%d, %p, %zd) failed",
               to_copy->input, buffer + position, position - sizeof(buffer));
        }
      } else if (read_bytes == 0 && to_copy->interface_address == NULL) {
        // read(2) says that this means EOF
        return NULL;
      }
      if (good_data) {
        position += read_bytes;
      }
    }

    CHECK_GE(position, 0);
    CHECK_LE(position, static_cast<ssize_t>(sizeof(buffer)));
    if (position > 0) {
      ssize_t sent_bytes = write(to_copy->output, buffer, position);
      if (sent_bytes == -1) {
        if (errno != EINTR) {
          PLOG(FATAL, "write(%d, %p, %zd) failed",
               to_copy->output, buffer, position);
        }
      } else if (sent_bytes != 0) {
        if (sent_bytes == position) {
          position = 0;
        } else {
          memmove(buffer, buffer + sent_bytes, position - sent_bytes);
          position -= sent_bytes;
        }
      }
    }
  }
}

int NetconsoleMain(int argc, char **argv) {
  WriteCoreDumps();
  logging::Init();
  logging::AddImplementation(new logging::StreamLogImplementation(stdout));

  int input, output;
  if (argc > 1) {
    output = open(argv[1], O_APPEND | O_CREAT | O_WRONLY | O_TRUNC, 0666);
    if (output == -1) {
      if (errno == EACCES || errno == ELOOP || errno == ENOSPC ||
          errno == ENOTDIR || errno == EROFS || errno == ETXTBSY) {
        PLOG(FATAL, "opening output file '%s' failed", argv[1]);
      }
      PLOG(FATAL, "open('%s', stuff, 0644) failed", argv[1]);
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
    PLOG(FATAL, "socket(AF_INET, SOCK_DGRAM, 0) failed");
  }
  if (setsockopt(from_crio, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
    PLOG(FATAL, "SOL_SOCKET::SO_REUSEADDR=%d(%d) failed", on, from_crio);
  }
  if (setsockopt(from_crio, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) == -1) {
    PLOG(FATAL, "SOL_SOCKET::SO_BROADCAST=%d(%d) failed", on, from_crio);
  }
  if (setsockopt(from_crio, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on)) == -1) {
    PLOG(FATAL, "IPROTO_IP::IP_PKTINFO=%d(%d) failed", on, from_crio);
  }
  union {
    struct sockaddr_in in;
    struct sockaddr addr;
  } address, crio_address;

  address.in.sin_family = AF_INET;
  address.in.sin_port = hton<uint16_t>(6666);
  address.in.sin_addr.s_addr = INADDR_ANY;

  crio_address.in.sin_family = AF_INET;
  crio_address.in.sin_port = hton<uint16_t>(0);
  crio_address.in.sin_addr = ::aos::configuration::GetOwnIPAddress();
  ::aos::util::SetLastSegment(&crio_address.in.sin_addr,
                              ::aos::NetworkAddress::kCRIO);

  if (bind(from_crio, &address.addr, sizeof(address)) == -1) {
    PLOG(FATAL, "bind(%d, %p, %zu) failed",
         from_crio, &address.addr, sizeof(address));
  }

  pthread_t input_thread, output_thread;

  address.in.sin_addr = ::aos::configuration::GetOwnIPAddress();
  ::aos::util::SetLastSegment(&address.in.sin_addr, NetworkAddress::kCRIO);
  fprintf(stderr, "Using cRIO IP %s.\n",
          inet_ntoa(address.in.sin_addr));

  if (input != -1) {
    int to_crio = socket(AF_INET, SOCK_DGRAM, 0);
    if (to_crio == -1) {
      PLOG(FATAL, "socket(AF_INET, SOCK_DGRAM, 0) failed");
    }
    if (setsockopt(to_crio, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
      PLOG(FATAL, "SOL_SOCKET::SO_REUSEADDR=%d(%d) failed", on, to_crio);
    }
    address.in.sin_port = hton<uint16_t>(6668);
    if (connect(to_crio, &address.addr, sizeof(address)) == -1) {
      PLOG(FATAL, "connect(%d, %p, %zu) failed",
           to_crio, &address.addr, sizeof(address));
    }
    FDsToCopy input_fds{input, to_crio, nullptr, nullptr};
    if (pthread_create(&input_thread, NULL, FDCopyThread, &input_fds) == -1) {
      PLOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed",
           &input_thread, FDCopyThread, &input_fds);
    }
  }

  address.in.sin_addr = ::aos::configuration::GetOwnIPAddress();
  FDsToCopy output_fds{from_crio, output, &address.in, &crio_address.in};
  if (pthread_create(&output_thread, NULL, FDCopyThread, &output_fds) == -1) {
    PLOG(FATAL, "pthread_create(%p, NULL, %p, %p) failed",
         &output_thread, FDCopyThread, &output_fds);
  }

  // input_thread will finish when stdin gets an EOF
  if (pthread_join((input == -1) ? output_thread : input_thread, NULL) == -1) {
    PLOG(FATAL, "pthread_join(a_thread, NULL) failed");
  }
  exit(EXIT_SUCCESS);
}

}  // namespace
}  // namespace aos

int main(int argc, char **argv) {
  return ::aos::NetconsoleMain(argc, argv);
}
