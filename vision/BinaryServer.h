#ifndef FRC971_VISION_BINARY_SERVER_H_
#define FRC971_VISION_BINARY_SERVER_H_

#include <sys/types.h> 
#include <sys/socket.h>

#include "event2/buffer.h"
#include "event2/event.h"
#include "event2/listener.h"
#include "event2/bufferevent.h"

#include "aos/common/mutex.h"

#include "vision/PacketNotifier.h"

namespace frc971 {
namespace vision {

/* This runs the libevent loop and interfaces in with the sockets provided from the PacketNotifier
 * to allow a secondary process to focus primarily on doing processing and then feeding this task.
 */
class BinaryServer {
 public:
  BinaryServer(uint16_t port,PacketNotifier *notify);


  void StartServer(uint16_t port);
 private:
  event_base *const _base;
  int _fd;
  PacketNotifier *_notify;
  bool have_id;
  int _client_fd;
  struct event *frame_notify;
  struct evbuffer *_output;
  struct bufferevent *_bufev;
  struct evconnlistener *listener;
  void Accept(evconnlistener *listener, evutil_socket_t fd,
      struct sockaddr* /*address*/, int /*socklen*/);
  void Notify(int /*fd*/, short /*what*/);
  void ErrorEvent(struct bufferevent *bev,short events);
  static void StaticAccept(evconnlistener *listener, evutil_socket_t fd,
          struct sockaddr *address, int socklen,void *self){
    ((BinaryServer *)(self))->Accept(listener, fd, address, socklen);
  }
  static void StaticNotify(int fd, short what, void *self){
    ((BinaryServer *)(self))->Notify(fd, what);
  }
  static void StaticErrorEvent(struct bufferevent *bev,short events,void *self){
    ((BinaryServer *)(self))->ErrorEvent(bev,events);
  }
};

}  // namespace vision
}  // namespace frc971

#endif  // FRC971_VISION_BINARY_SERVER_H_
