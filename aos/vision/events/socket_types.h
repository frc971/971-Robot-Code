#ifndef _AOS_VISION_EVENTS_SOCKET_TYPES_H_
#define _AOS_VISION_EVENTS_SOCKET_TYPES_H_

#include <poll.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "aos/vision/events/tcp_server.h"
#include "aos/vision/image/image_types.h"
#include "google/protobuf/message.h"

namespace aos {
namespace events {

// Simple TCP client connection that sends messages prefixed by length.
// Useful to broadcast to a message all connected clients.
class DataSocket : public events::SocketConnection {
 public:
  // Aliasing cast.
  union data_len {
    uint32_t len;
    char buf[4];
  };

  DataSocket(events::TCPServerBase *serv, int fd)
      : events::SocketConnection(serv, fd) {}

  ~DataSocket() { printf("Closed connection on descriptor %d\n", fd()); }

  void ReadEvent() override {
    // Ignore reads, but don't leave them pending.
    ssize_t count;
    char buf[512];
    while (true) {
      count = read(fd(), &buf, sizeof buf);
      if (count <= 0) {
        if (errno != EAGAIN) {
          CloseConnection();
          return;
        }
      }
    }
  }

  void Emit(const google::protobuf::Message &data) {
    std::string d;
    if (data.SerializeToString(&d)) {
      Emit(d);
    }
  }

  void Emit(vision::DataRef data) {
    data_len len;
    len.len = data.size();
    int res = send(fd(), len.buf, sizeof len.buf, MSG_NOSIGNAL);
    if (res == -1) {
      CloseConnection();
      return;
    }
    size_t write_count = 0;
    while (write_count < data.size()) {
      int len = send(fd(), &data.data()[write_count], data.size() - write_count,
                     MSG_NOSIGNAL);
      if (len == -1) {
        if (errno == EAGAIN) {
          struct pollfd waiting;
          waiting.fd = fd();
          waiting.events = POLLOUT;
          poll(&waiting, 1, -1);
        } else {
          CloseConnection();
          return;
        }
      } else {
        write_count += len;
      }
      if (write_count != data.size()) printf("wrote: %d\n", len);
    }
  }

 private:
  void CloseConnection() {
    loop()->Delete(this);
    close(fd());
    delete this;
  }
};

}  // namespace events
}  // namespace aos

#endif  // _AOS_VISION_EVENTS_SOCKET_TYPES_H_
