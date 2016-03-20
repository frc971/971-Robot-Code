#ifndef AOS_VISION_EVENTS_EPOLL_EVENTS_H_
#define AOS_VISION_EVENTS_EPOLL_EVENTS_H_

#include <stdint.h>
#include <limits.h>

#include <memory>

#include "aos/common/time.h"

namespace aos {
namespace events {

class EpollLoop;

// Performs an asychronous wait using an EpollLoop.
//
// Note: this does not have very high resolution (sub-millisecond).
class EpollWait {
 public:
  virtual ~EpollWait() {}

  // Called when the currently set time is reached.
  virtual void Done() = 0;

  // Sets this wait to end at new_time.
  // A negative new_time disables this wait.
  void SetTime(const ::aos::time::Time &new_time) { time_ = new_time; }

 private:
  // Calculates how long to wait starting at now and calls Done() if
  // appropriate.
  // Returns the number of milliseconds from now that this event will expire in.
  // Returns -1 if this wait is never going to expire.
  // Returns INT_MAX if this wait expires in longer than that.
  int Recalculate(const ::aos::time::Time &now) {
    if (time_ < ::aos::time::Time::kZero) return -1;
    if (time_ <= now) {
      Done();
      time_ = ::aos::time::Time(-1, 0);
      return -1;
    }
    if (time_ - now > ::aos::time::Time::InMS(INT_MAX)) {
      return INT_MAX;
    } else {
      return (time_ - now).ToMSec();
    }
  }

  ::aos::time::Time time_ = ::aos::time::Time::kZero;

  friend class EpollLoop;
};

// Represents a file descriptor which signals events from an EpollLoop.
class EpollEvent {
 public:
  EpollEvent(int fd) : fd_(fd) {}
  virtual ~EpollEvent() {}

  int fd() { return fd_; }

  // Called when fd() is readable. This must make fd() non-readable or the event
  // loop degrades into a busy loop.
  virtual void ReadEvent() = 0;

 private:
  const int fd_;
};

// Provides a way for code to be notified every time after events are handled by
// an EpollLoop. This is mainly a hack for the GTK integration and testing;
// think very carefully before using it anywhere else.
class EpollWatcher {
 public:
  virtual ~EpollWatcher() {}

  // Called after events have been processed each time the event loop wakes up.
  virtual void Wake() = 0;
};

// A file descriptor based event loop implemented with epoll.
//
// There is currently no way to remove events because that's hard
// (have to deal with events that come in for a file descriptor after it's
// removed, which means not closing the fd or destroying the object until the
// epoll mechanism confirms its removal) and we don't have a use for it.
class EpollLoop {
 public:
  EpollLoop();

  // Ways to add various objects which interact with this event loop.
  // None of these take ownership of the passed-in objects.
  void AddWait(EpollWait *wait);
  void Add(EpollEvent *event);
  void AddWatcher(EpollWatcher *watcher);

  // Loops forever, handling events.
  void Run();

 private:
  class Impl;

  ::std::unique_ptr<Impl> impl_;
};

}  // namespace events
}  // namespace aos

#endif  // AOS_VISION_EVENTS_EPOLL_EVENTS_H_
