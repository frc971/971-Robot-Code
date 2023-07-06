#ifndef AOS_EVENTS_EVENT_LOOP_EVENT_H
#define AOS_EVENTS_EVENT_LOOP_EVENT_H

#include "glog/logging.h"

#include "aos/time/time.h"

namespace aos {

// Common interface to track when callbacks and timers should have happened.
class EventLoopEvent {
 public:
  virtual ~EventLoopEvent() {}

  bool valid() const { return event_time_ != monotonic_clock::max_time; }
  void Invalidate() {
    event_time_ = monotonic_clock::max_time;
    generation_ = 0;
  }

  monotonic_clock::time_point event_time() const {
    DCHECK(valid());
    return event_time_;
  }
  void set_event_time(monotonic_clock::time_point event_time) {
    event_time_ = event_time;
  }

  // Internal book-keeping for EventLoop.
  size_t generation() const {
    DCHECK(valid());
    return generation_;
  }
  void set_generation(size_t generation) { generation_ = generation; }

  virtual void HandleEvent() noexcept = 0;

 private:
  monotonic_clock::time_point event_time_ = monotonic_clock::max_time;
  size_t generation_ = 0;
};

// Adapter class to implement EventLoopEvent by calling HandleEvent on T.
template <typename T>
class EventHandler final : public EventLoopEvent {
 public:
  EventHandler(T *t) : t_(t) {}
  ~EventHandler() override = default;
  void HandleEvent() noexcept override { t_->HandleEvent(); }

 private:
  T *const t_;
};

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_EVENT_H
