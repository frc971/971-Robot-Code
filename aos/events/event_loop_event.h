#ifndef AOS_EVENTS_EVENT_LOOP_EVENT_H
#define AOS_EVENTS_EVENT_LOOP_EVENT_H

#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos {

// Common interface to track when callbacks and timers should have happened.
class EventLoopEvent {
 public:
  virtual ~EventLoopEvent() {}

  bool valid() const { return event_time_ != monotonic_clock::max_time; }
  void Invalidate() { event_time_ = monotonic_clock::max_time; }

  monotonic_clock::time_point event_time() const {
    DCHECK(valid());
    return event_time_;
  }

  virtual void HandleEvent() = 0;

  void set_event_time(monotonic_clock::time_point event_time) {
    event_time_ = event_time;
  }

 private:
  monotonic_clock::time_point event_time_ = monotonic_clock::max_time;
};

// Adapter class to implement EventLoopEvent by calling HandleEvent on T.
template <typename T>
class EventHandler final : public EventLoopEvent {
 public:
  EventHandler(T *t) : t_(t) {}
  ~EventHandler() = default;
  void HandleEvent() override { t_->HandleEvent(); }

 private:
  T *const t_;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_EVENT_H
