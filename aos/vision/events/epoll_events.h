#ifndef AOS_VISION_EVENTS_EPOLL_EVENTS_H_
#define AOS_VISION_EVENTS_EPOLL_EVENTS_H_

#include <limits.h>
#include <stdint.h>
#include <memory>
#include <vector>

#include "aos/common/scoped_fd.h"
#include "aos/common/time.h"

namespace aos {
namespace events {

class EpollLoop;

// Performs an asychronous wait using an EpollLoop.
//
// Note: this does not have very high resolution (sub-millisecond).
// TODO(parker): This is mostly broken.
class EpollWait {
 public:
  virtual ~EpollWait() {}

  // Called when the currently set time is reached.
  virtual void Done() = 0;

  // Sets this wait to end at new_time.
  // A negative new_time disables this wait.
  void SetTime(const monotonic_clock::time_point new_time) { time_ = new_time; }

 private:
  // Calculates how long to wait starting at now and calls Done() if
  // appropriate.
  // Returns the number of milliseconds from now that this event will expire in.
  // Returns -1 if this wait is never going to expire.
  // Returns INT_MAX if this wait expires in longer than that.
  int Recalculate(const monotonic_clock::time_point now) {
    if (time_ < monotonic_clock::epoch()) return -1;
    if (time_ <= now) {
      time_ = monotonic_clock::time_point(::std::chrono::seconds(-1));
      Done();
    }
    // Duplicate above to allow Done to change itself.
    if (time_ < monotonic_clock::epoch()) return -1;
    if (time_ <= now) {
      return -1;
    }

    if (time_ - now > ::std::chrono::milliseconds(INT_MAX)) {
      return INT_MAX;
    } else {
      return ::std::chrono::duration_cast<::std::chrono::milliseconds>(time_ -
                                                                       now)
          .count();
    }
  }

  ::aos::monotonic_clock::time_point time_ = ::aos::monotonic_clock::epoch();

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

  EpollLoop *loop() { return loop_; }

 private:
  const int fd_;
  friend class EpollLoop;
  EpollLoop *loop_ = nullptr;
};

// A file descriptor based event loop implemented with epoll.
class EpollLoop {
 public:
  EpollLoop();

  // Ways to add various objects which interact with this event loop.
  // None of these take ownership of the passed-in objects.
  void AddWait(EpollWait *wait);
  void Add(EpollEvent *event);

  // Delete event. Note that there are caveats here as this is
  // not idiot proof.
  // ie:
  //  - Do not call from other threads.
  //  - Do not free while the object could still receive events.
  //  - This is safe only because the events are set as edge.
  //  TODO(parker): The thread-safety of this should be investigated and
  //  improved as well as adding support for non-edge events if this is to
  //  be used more generally.
  void Delete(EpollEvent *event);

  // Loops forever, handling events.
  void Run();

  // Fuses with gtk_main().
  // Note that the dep for this is separate: //aos/vision/events:gtk_event
  void RunWithGtkMain();

 private:
  int epoll_fd() { return epoll_fd_.get(); }

  int CalculateTimeout();

  ::aos::ScopedFD epoll_fd_;
  ::std::vector<EpollWait *> waits_;
};

}  // namespace events
}  // namespace aos

#endif  // AOS_VISION_EVENTS_EPOLL_EVENTS_H_
