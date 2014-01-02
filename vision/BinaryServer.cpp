#include "vision/BinaryServer.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <vector>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "aos/externals/libjpeg/include/jpeglib.h"
#include "aos/atom_code/camera/Buffers.h"
#include "aos/common/time.h"

namespace frc971 {
namespace vision {

static void echo_read_cb(struct bufferevent *bev, void * /*ctx*/) {
  struct evbuffer *input = bufferevent_get_input(bev);
  struct evbuffer *output = bufferevent_get_output(bev);

  size_t len = evbuffer_get_length(input);
  char *data;
  data = (char *)malloc(len);
  evbuffer_copyout(input, data, len);

  printf("we got some data: %s\n", data);

  evbuffer_add_buffer(output, input);
}

void BinaryServer::ErrorEvent(struct bufferevent *bev, short events) {
  if (events & BEV_EVENT_ERROR) perror("Error from bufferevent");
  if (events & (BEV_EVENT_EOF | BEV_EVENT_ERROR)) {
    have_id = false;
    bufferevent_free(bev);
  }
}

void BinaryServer::Accept(struct evconnlistener *listener, evutil_socket_t fd,
                          struct sockaddr * /*address*/, int /*socklen*/) {
  struct event_base *base = evconnlistener_get_base(listener);
  if (!have_id) {
    struct bufferevent *bev =
        bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
    _output = bufferevent_get_output(bev);
    _bufev = bev;
    have_id = true;
    int no_delay_flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay_flag,
               sizeof(no_delay_flag));

    bufferevent_setcb(bev, echo_read_cb, NULL, StaticErrorEvent, this);

    bufferevent_enable(bev, EV_READ | EV_WRITE);
  }
}
static void accept_error_cb(struct evconnlistener *listener, void * /*ctx*/) {
  struct event_base *base = evconnlistener_get_base(listener);
  int err = EVUTIL_SOCKET_ERROR();
  fprintf(stderr, "Got an error %d (%s) on the listener. "
                  "Shutting down.\n",
          err, evutil_socket_error_to_string(err));

  event_base_loopexit(base, NULL);
}

void BinaryServer::StartServer(uint16_t port) {
  _fd = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = htons(port);
  sin.sin_addr.s_addr = inet_addr("0.0.0.0");

  listener = evconnlistener_new_bind(
      _base, StaticAccept, this, LEV_OPT_CLOSE_ON_FREE | LEV_OPT_REUSEABLE, -1,
      (struct sockaddr *)(void *)&sin, sizeof(sin));

  if (!listener) {
    fprintf(stderr, "%s:%d: Couldn't create listener\n", __FILE__, __LINE__);
    exit(-1);
  }

  evconnlistener_set_error_cb(listener, accept_error_cb);
}

void BinaryServer::Notify(int fd, short /*what*/) {
  char notes[4096];
  int count = read(fd, notes, 4096);
  if (count == 0) {
    close(fd);
    fprintf(stderr, "%s:%d: Error No cheeze from OpenCV task!!!\n", __FILE__,
            __LINE__);
    exit(-1);
  }
  printf("notified!: %d\n", count);
  if (have_id) {
    printf("got someone to read my stuff!\n");
    char *binary_data;
    size_t len;
    if (_notify->GetData(&binary_data, &len)) {
      printf("here is for sending\n");
      evbuffer_add_reference(_output, binary_data, len,
                             PacketNotifier::StaticDataSent, _notify);
      printf("this is how sending went %d\n",
             bufferevent_flush(_bufev, EV_WRITE, BEV_FLUSH));
    }
  }
}

// Constructor
BinaryServer::BinaryServer(uint16_t port,
                           frc971::vision::PacketNotifier *notify)
    : _base(event_base_new()) {
  have_id = false;
  StartServer(port);
  _notify = notify;
  frame_notify = event_new(_base, notify->RecieverFD(), EV_READ | EV_PERSIST,
                           StaticNotify, this);
  event_add(frame_notify, NULL);
  event_base_dispatch(_base);
}

}  // namespace vision
}  // namespace frc971
