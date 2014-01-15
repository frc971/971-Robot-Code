#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <inttypes.h>

#include <vector>

#include "aos/common/network_port.h"
#include "aos/linux_code/init.h"
#include "aos/linux_code/camera/Buffers.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace camera {

namespace {

// doesn't like being a static class member
static const unsigned char dht_table[] = {
  0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
  0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x01, 0x00, 0x03,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
  0x0a, 0x0b, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05,
  0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01, 0x02, 0x03, 0x00, 0x04,
  0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
  0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15,
  0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95,
  0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
  0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2,
  0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5,
  0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
  0xfa, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05,
  0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
  0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22,
  0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
  0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
  0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
  0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
  0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94,
  0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
  0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
  0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
  0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7,
  0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};

const char kFirstHeader[] = "HTTP/1.0 200 OK\r\n"
"Connection: close\r\n"
"Server: AOS/0.0 Camera\r\n"
"Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, "
"post-check=0, max-age=0\r\n"  // this and above from mjpg-streamer
"Pragma: no-cache\r\n"
"Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"  // also from mjpg-streamer
"Content-Type: multipart/x-mixed-replace; boundary=boundarydonotcross\r\n";

}  // namespace

class HTTPStreamer {
  // Represents a single client. Handles all reading and writing of sockets and
  // queues.
  class Client {
    enum class State {
      kReadHeaders,
      kWriteHeaders,
      kWriteBoundary,  // these last 2 loop to each other
      kWriteImage,
      kWriteDHT,  // happens in the middle of kWriteImage
    };
    const int sock_;
    State state_;
    inline fd_set *GetFDSetForCurrentState(fd_set *read_fds,
                                           fd_set *write_fds) {
      if (state_ == State::kReadHeaders) {
        return read_fds;
      } else {
        return write_fds;
      }
    }
    // MUST BE LONG ENOUGH TO HOLD kBoundaryText WITH A BIGGISH # IN IT
    char scratch_[4096];
    int to_write_;
    int zero_reads_;
    static const int kMaxZeroReads = 2000;
    size_t pos_, dht_pos_, dht_start_;
    Buffers buffers_;
    const void *current_;
    uint32_t size_;

    Client(const Client &);
    void operator=(const Client &);

   public:
    explicit Client(int sock) : sock_(sock), state_(State::kReadHeaders),
        zero_reads_(0), pos_(0) {}
    ~Client() {
      LOG(DEBUG, "closing socket %d\n", sock_);
      if (close(sock_) == -1) {
        LOG(INFO, "closing socket %d for destruction failed with %d: %s\n",
            sock_, errno, strerror(errno));
      }
    }
    // Set any fds necessary into the 2 arguments.
    void FDSet(fd_set *read_fds, fd_set *write_fds) {
      FD_SET(sock_, GetFDSetForCurrentState(read_fds, write_fds));
    }
    // The arguments are the same as the last FDSet call (after a successful
    // select).
    // Return value is whether or not to keep this one around.
    bool Process(fd_set *read_fds, fd_set *write_fds) {
      // if the socket we're waiting on isn't ready
      if (!FD_ISSET(sock_, GetFDSetForCurrentState(read_fds, write_fds))) {
        return true;
      }

      ssize_t ret;
      switch (state_) {
        case State::kReadHeaders:
          if (pos_ >= sizeof(scratch_)) {
            LOG(WARNING, "read too many bytes of headers on sock %d."
                " somebody should increase the size of scratch_\n", sock_);
            return false;
          }
          if (zero_reads_ > kMaxZeroReads) {
            LOG(WARNING, "read 0 bytes %d times on sock %d. giving up\n",
                zero_reads_, sock_);
            return false;
          }
          ret = read(sock_, scratch_ + pos_, sizeof(scratch_) - pos_);
          if (ret == -1) {
            LOG(WARNING, "read(%d, %p, %zd) failed with %d: %s\n",
                sock_, scratch_ + pos_, sizeof(scratch_) - pos_,
                errno, strerror(errno));
            return false;
          }
          pos_ += ret;
          // if we just received \r\n\r\n (the end of the headers)
          if (scratch_[pos_ - 4] == '\r' && scratch_[pos_ - 3] == '\n' &&
              scratch_[pos_ - 2] == '\r' && scratch_[pos_ - 1] == '\n') {
            LOG(INFO, "entering state kWriteHeaders"
                " after %zd bytes of headers read\n", pos_ - 1);
            pos_ = 0;
            state_ = State::kWriteHeaders;
          }
          scratch_[pos_] = '\0';
          if (ret == 0) {
            ++zero_reads_;
          } else {
            zero_reads_ = 0;
            LOG(DEBUG, "read %zd bytes of headers scratch_=%s\n",
                ret, scratch_);
          }
          break;
        case State::kWriteHeaders:
          // this intentionally doesn't write the terminating \0 on the string
          ret = write(sock_, kFirstHeader + pos_, sizeof(kFirstHeader) - pos_);
          if (ret == -1) {
            LOG(WARNING, "write(%d, %p, %zd) failed with %d: %s\n",
                sock_, kFirstHeader + pos_, sizeof(kFirstHeader) - pos_,
                errno, strerror(errno));
          } else {
            pos_ += ret;
            if (pos_ >= sizeof(kFirstHeader)) {
              current_ = NULL;
              state_ = State::kWriteBoundary;
            }
          }
          break;
        case State::kWriteBoundary:
          if (current_ == NULL) {
            timeval timestamp;
            current_ = buffers_.GetNext(false, &size_, &timestamp, NULL);

            /*static int skip = 0;
            if (current_ != NULL) skip = (skip + 1) % 30;
            if (!skip) current_ = NULL;
            if (current_ == NULL) break;*/

#if 0
            // set pos_ to where the first header starts
            for (pos_ = 0; static_cast<const uint8_t *>(current_)[pos_] != 0xFF;
                 ++pos_);
#else
            pos_ = 0;
#endif
#if 0
            // go through the frame looking for the start of frame marker
            for (dht_start_ = 0;
                 static_cast<const uint8_t *>(current_)[dht_start_ + 0] !=
                 0xFF &&
                 static_cast<const uint8_t *>(current_)[dht_start_ + 1] !=
                 0xC0 &&
                 dht_start_ < size_; ++dht_start_)
              printf("[%zd]=%"PRIx8" ", dht_start_,
                     static_cast<const uint8_t *>(current_)[dht_start_]);
            if (dht_start_ >= size_) {
              LOG(WARNING, "couldn't find start of frame marker\n");
              return false;
            }
#else
            dht_start_ = 0;
#endif
            dht_pos_ = 0;

            // aos.ChannelImageGetter depends on the exact format of this
            to_write_ = snprintf(scratch_, sizeof(scratch_),
                                "\r\n--boundarydonotcross\r\n"
                                "Content-Type: image/jpeg\r\n"
                                "Content-Length: %" PRId32 "\r\n"
                                "X-Timestamp: %ld.%06ld\r\n"
                                "\r\n",
                                size_,
                                timestamp.tv_sec, timestamp.tv_usec);
          }
          ret = write(sock_, scratch_ + pos_, to_write_ - pos_);
          if (ret == -1) {
            LOG(WARNING, "write(%d, %p, %zd) failed with %d: %s\n",
                sock_, scratch_ + pos_, to_write_ - pos_,
                errno, strerror(errno));
            return false;
          } else {
            pos_ += ret;
            if (static_cast<ssize_t>(pos_) >= to_write_) {
              pos_ = 0;
              state_ = State::kWriteImage;
            }
          }
          break;
        case State::kWriteImage:
          ret = write(sock_, static_cast<const char *>(current_) + pos_,
                      ((dht_start_ == 0) ? size_ : dht_start_) - pos_);
          if (ret == -1) {
            LOG(WARNING, "write(%d, %p, %zd) failed with %d: %s\n",
                sock_, static_cast<const char *>(current_) + pos_,
                ((dht_start_ == 0) ? size_ : dht_start_) - pos_,
                errno, strerror(errno));
            return false;
          } else {
            pos_ += ret;
            if (dht_start_ == 0) {
              if (pos_ >= size_) {
                buffers_.Release();
                current_ = NULL;
                state_ = State::kWriteBoundary;
              }
            } else {
              if (pos_ >= dht_start_) {
                dht_start_ = 0;
                state_ = State::kWriteDHT;
              }
            }
          }
          break;
        case State::kWriteDHT:
          ret = write(sock_, dht_table + dht_pos_,
                      sizeof(dht_table) - dht_pos_);
          if (ret == -1) {
            LOG(WARNING, "write(%d, %p, %zd) failed with %d: %s\n",
                sock_, dht_table + dht_pos_, sizeof(dht_table) - dht_pos_,
                errno, strerror(errno));
            return false;
          } else {
            dht_pos_ += ret;
            if (dht_pos_ >= sizeof(dht_table)) {
              state_ = State::kWriteImage;
            }
          }
          break;
        default:
          LOG(FATAL, "something weird happened\n");
      }
      return true;
    }
  };

  const int bind_socket_;

 public:
  HTTPStreamer() : bind_socket_(socket(AF_INET, SOCK_STREAM, 0)) {
    if (bind_socket_ < 0) {
      LOG(FATAL, "socket(AF_INET, SOCK_STREAM, 0) failed with %d: %s\n",
          errno, strerror(errno));
    }

    union {
      sockaddr_in in;
      sockaddr addr;
    } bind_sockaddr;
    memset(&bind_sockaddr, 0, sizeof(bind_sockaddr));
    bind_sockaddr.in.sin_family = AF_INET;
    bind_sockaddr.in.sin_port =
        htons(static_cast<uint16_t>(aos::NetworkPort::kCameraStreamer));
    bind_sockaddr.in.sin_addr.s_addr = htonl(INADDR_ANY);
    int optval = 1;
    setsockopt(bind_socket_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    if (bind(bind_socket_, &bind_sockaddr.addr,
             sizeof(bind_sockaddr.addr)) == -1) {
      LOG(FATAL, "bind(%d, %p) failed because of %d: %s\n",
          bind_socket_, &bind_sockaddr.addr, errno, strerror(errno));
    }

    if (listen(bind_socket_, 10) == -1) {
      LOG(FATAL, "listen(%d, 10) failed because of %d: %s\n", bind_socket_,
          errno, strerror(errno));
    }
    const int flags = fcntl(bind_socket_, F_GETFL, 0);
    if (flags == -1) {
      LOG(FATAL, "fcntl(%d, F_GETFL, 0) failed because of %d: %s\n",
          bind_socket_, errno, strerror(errno));
    }
    if (fcntl(bind_socket_, F_SETFL, flags | O_NONBLOCK) == -1) {
      LOG(FATAL, "fcntl(%d, F_SETFL, %x) failed because of %d: %s\n",
          bind_socket_, flags | O_NONBLOCK, errno, strerror(errno));
    }
  }
  void Run() {
    signal(SIGPIPE, SIG_IGN);

    std::vector<Client *> clients;
    fd_set read_fds, write_fds;
    while (true) {
      FD_ZERO(&read_fds);
      FD_ZERO(&write_fds);
      FD_SET(bind_socket_, &read_fds);
      for (auto it = clients.begin(); it != clients.end(); ++it) {
        (*it)->FDSet(&read_fds, &write_fds);
      }
      switch (select(FD_SETSIZE, &read_fds, &write_fds,
                     NULL,  // err
                     NULL)) {  // timeout
        case -1:
          LOG(ERROR, "select(FD_SETSIZE(=%d), %p, %p, NULL, NULL) failed"
              " because of %d: %s\n", FD_SETSIZE, &read_fds, &write_fds,
              errno, strerror(errno));
          continue;
        case 0:
          LOG(ERROR, "select with NULL timeout timed out...\n");
          continue;
      }

      if (FD_ISSET(bind_socket_, &read_fds)) {
        const int sock = accept4(bind_socket_, NULL, NULL, SOCK_NONBLOCK);
        if (sock == -1) {
          LOG(ERROR, "accept4(%d, NULL, NULL, SOCK_NONBLOCK(=%d) failed"
              " because of %d: %s\n",
              bind_socket_, SOCK_NONBLOCK, errno, strerror(errno));
        } else {
          clients.push_back(new Client(sock));
        }
      }

      std::vector<std::vector<Client *>::iterator> to_remove;
      for (auto it = clients.begin(); it != clients.end(); ++it) {
        if (!(*it)->Process(&read_fds, &write_fds)) {
          to_remove.push_back(it);
          delete *it;
        }
      }
      for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
        LOG(INFO, "removing client\n");
        clients.erase(*it);
      }
    }
  }
};

}  // namespace camera
}  // namespace aos

int main() {
  ::aos::InitNRT();
  ::aos::camera::HTTPStreamer streamer;
  streamer.Run();
  ::aos::Cleanup();
}
