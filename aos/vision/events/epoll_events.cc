#include "aos/vision/events/epoll_events.h"

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/epoll.h>

#include <vector>

#include "aos/common/scoped_fd.h"
#include "aos/common/logging/logging.h"

namespace aos {
namespace events {

class EpollLoop::Impl {
 public:
  Impl() : epoll_fd_(PCHECK(epoll_create1(0))) {}

  void Add(EpollEvent *event) {
    struct epoll_event temp_event;
    temp_event.data.ptr = static_cast<void *>(event);
    temp_event.events = EPOLLIN;
    PCHECK(epoll_ctl(epoll_fd(), EPOLL_CTL_ADD, event->fd(), &temp_event));
  }

  void Run() {
    while (true) {
      const int timeout = CalculateTimeout();
      static constexpr size_t kNumberOfEvents = 64;
      epoll_event events[kNumberOfEvents];
      const int number_events = PCHECK(
          epoll_wait(epoll_fd(), events, kNumberOfEvents, timeout));

      for (int i = 0; i < number_events; i++) {
        EpollEvent *event =
            static_cast<EpollEvent *>(events[i].data.ptr);
        if ((events[i].events & ~(EPOLLIN | EPOLLPRI)) != 0) {
          LOG(FATAL, "unexpected epoll events set in %x on %d\n",
              events[i].events, event->fd());
        }
        event->ReadEvent();
      }

      for (EpollWatcher *watcher : watchers_) {
        watcher->Wake();
      }
    }
  }

  void AddWait(EpollWait *wait) { waits_.push_back(wait); }
  void AddWatcher(EpollWatcher *watcher) { watchers_.push_back(watcher); }

  int epoll_fd() { return epoll_fd_.get(); }

 private:
  // Calculates the new timeout value to pass to epoll_wait.
  int CalculateTimeout() {
    const ::aos::time::Time now = ::aos::time::Time::Now();
    int r = -1;
    for (EpollWait *c : waits_) {
      const int new_timeout = c->Recalculate(now);
      if (new_timeout >= 0) {
        if (r < 0 || new_timeout < r) {
          r = new_timeout;
        }
      }
    }
    return r;
  }

 private:
  ::aos::ScopedFD epoll_fd_;
  ::std::vector<EpollWait *> waits_;
  ::std::vector<EpollWatcher *> watchers_;
};

EpollLoop::EpollLoop() : impl_(new Impl()) {}
void EpollLoop::Run() { impl_->Run(); }
void EpollLoop::Add(EpollEvent *event) { impl_->Add(event); }
void EpollLoop::AddWait(EpollWait *wait) { impl_->AddWait(wait); }
void EpollLoop::AddWatcher(EpollWatcher *watcher) {
  impl_->AddWatcher(watcher);
}

}  // namespace events
}  // namespace aos
