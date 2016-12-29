#ifndef _AOS_VISION_EVENTS_TCP_SERVER_H_
#define _AOS_VISION_EVENTS_TCP_SERVER_H_

#include "aos/vision/events/epoll_events.h"
#include "aos/vision/events/intrusive_free_list.h"

#include <memory>
#include <vector>

namespace aos {
namespace events {

// Non-templatized base class of TCP server.
// TCPServer implements Construct which specializes the client connection
// based on the specific use-case.
template <class T>
class TCPServer;
class SocketConnection;
class TCPServerBase : public EpollEvent {
 public:
  TCPServerBase(int fd) : EpollEvent(fd) {}
  ~TCPServerBase();

 protected:
  // Listens on port portno. File descriptor to
  // accept on is returned.
  static int SocketBindListenOnPort(int portno);

 private:
  virtual SocketConnection *Construct(int child_fd) = 0;
  void ReadEvent() override;
  friend class SocketConnection;
  template <class T>
  friend class TCPServer;
  intrusive_free_list<SocketConnection> free_list;
};

// Base class for client connections. Clients are responsible for
// deleting themselves once the connection is broken. This will remove
// the entry from the free list.
class SocketConnection : public EpollEvent,
                         public intrusive_free_list<SocketConnection>::element {
 public:
  SocketConnection(TCPServerBase *server, int fd)
      : EpollEvent(fd), element(&server->free_list, this) {}
};

// T should be a subclass of SocketConnection.
template <class T>
class TCPServer : public TCPServerBase {
 public:
  TCPServer(int port) : TCPServerBase(SocketBindListenOnPort(port)) {}
  SocketConnection *Construct(int child_fd) override {
    return new T(this, child_fd);
  }

  static std::unique_ptr<TCPServer<T>> Make(int port) {
    return std::unique_ptr<TCPServer<T>>(new TCPServer<T>(port));
  }

  // Call blk on each entry of the free-list. This is used to send a message
  // to all clients.
  template <typename EachBlock>
  void Broadcast(const EachBlock &blk) {
    auto a = free_list.begin();
    while (a) {
      auto client = static_cast<T *>(a);
      blk(client);
      a = a->next();
    }
  }
};

}  // namespace events
}  // namespace aos

#endif  // _AOS_VISION_EVENTS_TCP_SERVER_H_
