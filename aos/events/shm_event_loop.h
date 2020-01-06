#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <vector>

#include "aos/events/epoll.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"

namespace aos {
namespace internal {

class WatcherState;
class TimerHandlerState;
class PhasedLoopHandler;
class ShmSender;
class ShmFetcher;

}  // namespace internal

// Specialization of EventLoop that is built from queues running out of shared
// memory.
//
// TODO(austin): Timing reports break multiple threads.  Need to add back in a
// mutex.
// This object must be interacted with from one thread, but the Senders
// and Fetchers may be used from multiple threads afterwords (as long as their
// destructors are called back in one thread again)
class ShmEventLoop : public EventLoop {
 public:
  ShmEventLoop(const Configuration *configuration);
  ~ShmEventLoop() override;

  // Runs the event loop until Exit is called, or ^C is caught.
  void Run();
  // Exits the event loop.  Async safe.
  void Exit();

  aos::monotonic_clock::time_point monotonic_now() override {
    return aos::monotonic_clock::now();
  }
  aos::realtime_clock::time_point realtime_now() override {
    return aos::realtime_clock::now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) override;
  std::unique_ptr<RawFetcher> MakeRawFetcher(const Channel *channel) override;

  void MakeRawWatcher(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> watcher)
      override;

  TimerHandler *AddTimer(std::function<void()> callback) override;
  aos::PhasedLoopHandler *AddPhasedLoop(
      std::function<void(int)> callback,
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset =
          std::chrono::seconds(0)) override;

  void OnRun(std::function<void()> on_run) override;

  void SetRuntimeRealtimePriority(int priority) override;

  void set_name(const std::string_view name) override {
    name_ = std::string(name);
    UpdateTimingReport();
  }
  const std::string_view name() const override { return name_; }
  const Node *node() const override { return node_; }

  int priority() const override { return priority_; }

  internal::EPoll *epoll() { return &epoll_; }

 private:
  friend class internal::WatcherState;
  friend class internal::TimerHandlerState;
  friend class internal::PhasedLoopHandler;
  friend class internal::ShmSender;
  friend class internal::ShmFetcher;

  void HandleEvent();

  // Tracks that we can't have multiple watchers or a sender and a watcher (or
  // multiple senders) on a single queue (path).
  void Take(const Channel *channel);

  // Returns the TID of the event loop.
  pid_t GetTid() override;

  std::vector<std::function<void()>> on_run_;
  int priority_ = 0;
  std::string name_;
  const Node *const node_;
  std::vector<std::string> taken_;

  internal::EPoll epoll_;
};


}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
