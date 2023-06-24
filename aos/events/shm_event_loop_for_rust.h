#ifndef AOS_EVENTS_SHM_EVENT_LOOP_FOR_RUST_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_FOR_RUST_H_

#include <memory>

#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"

namespace aos {

class ShmEventLoopForRust {
 public:
  ShmEventLoopForRust(const Configuration *configuration)
      : event_loop_(configuration) {}

  const EventLoop *event_loop() const { return &event_loop_; }
  EventLoop *event_loop_mut() { return &event_loop_; }

  std::unique_ptr<ExitHandle> MakeExitHandle() {
    return event_loop_.MakeExitHandle();
  }

  void Run() { event_loop_.Run(); }

 private:
  ShmEventLoop event_loop_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_FOR_RUST_H_
