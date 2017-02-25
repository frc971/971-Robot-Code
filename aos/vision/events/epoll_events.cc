#include "aos/vision/events/epoll_events.h"

#include <fcntl.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#include "aos/common/logging/logging.h"

namespace aos {
namespace events {

EpollLoop::EpollLoop() : epoll_fd_(PCHECK(epoll_create1(0))) {}

void EpollLoop::Add(EpollEvent *event) {
  event->loop_ = this;
  struct epoll_event temp_event;
  temp_event.data.ptr = static_cast<void *>(event);
  temp_event.events = EPOLLIN;
  PCHECK(epoll_ctl(epoll_fd(), EPOLL_CTL_ADD, event->fd(), &temp_event));
}

void EpollLoop::Delete(EpollEvent *event) {
  PCHECK(epoll_ctl(epoll_fd(), EPOLL_CTL_DEL, event->fd(), NULL));
}

void EpollLoop::Run() {
  while (true) {
    const int timeout = CalculateTimeout();
    static constexpr size_t kNumberOfEvents = 64;
    epoll_event events[kNumberOfEvents];
    const int number_events =
        PCHECK(epoll_wait(epoll_fd(), events, kNumberOfEvents, timeout));

    for (int i = 0; i < number_events; i++) {
      EpollEvent *event = static_cast<EpollEvent *>(events[i].data.ptr);
      if ((events[i].events & ~(EPOLLIN | EPOLLPRI | EPOLLERR)) != 0) {
        LOG(FATAL, "unexpected epoll events set in %x on %d\n",
            events[i].events, event->fd());
      }
      event->ReadEvent();
    }
  }
}

void EpollLoop::AddWait(EpollWait *wait) { waits_.push_back(wait); }

// Calculates the new timeout value to pass to epoll_wait.
int EpollLoop::CalculateTimeout() {
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  int r = -1;
  for (EpollWait *c : waits_) {
    const int new_timeout = c->Recalculate(monotonic_now);
    if (new_timeout >= 0) {
      if (r < 0 || new_timeout < r) {
        r = new_timeout;
      }
    }
  }
  return r;
}

}  // namespace events
}  // namespace aos
