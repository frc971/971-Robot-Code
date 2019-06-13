#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <unordered_set>
#include <vector>
#include "aos/condition.h"
#include "aos/mutex/mutex.h"
#include "aos/events/event-loop.h"

namespace aos {
namespace internal {

class WatcherThreadState;
class TimerHandlerState;

}  // namespace internal

// Specialization of EventLoop that is built from queues running out of shared
// memory. See more details at aos/queue.h
//
// This object must be interacted with from one thread, but the Senders and
// Fetchers may be used from multiple threads afterwords (as long as their
// destructors are called back in one thread again)
class ShmEventLoop : public EventLoop {
 public:
  ShmEventLoop();
  ~ShmEventLoop() override;

  ::aos::monotonic_clock::time_point monotonic_now() override {
    return ::aos::monotonic_clock::now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const std::string &path,
                                           const QueueTypeInfo &type) override;
  std::unique_ptr<RawFetcher> MakeRawFetcher(
      const std::string &path, const QueueTypeInfo &type) override;

  void MakeRawWatcher(
      const std::string &path, const QueueTypeInfo &type,
      std::function<void(const aos::Message *message)> watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override;

  void OnRun(std::function<void()> on_run) override;
  void Run() override;
  void Exit() override;

  void SetRuntimeRealtimePriority(int priority) override {
    if (is_running()) {
      ::aos::Die("Cannot set realtime priority while running.");
    }
    thread_state_->priority_ = priority;
  }

 private:
  friend class internal::WatcherThreadState;
  friend class internal::TimerHandlerState;
  // This ThreadState ensures that two watchers in the same loop cannot be
  // triggered concurrently.  Because watchers block threads indefinitely, this
  // has to be shared_ptr in case the EventLoop is destroyed before the thread
  // receives any new events.
  class ThreadState {
   public:
    void WaitForStart();

    bool is_running() { return loop_running_; }

    void Run();

    void Exit();

    void MaybeSetCurrentThreadRealtimePriority();

   private:
    friend class internal::WatcherThreadState;
    friend class internal::TimerHandlerState;
    friend class ShmEventLoop;

    // This mutex ensures that only one watch event happens at a time.
    aos::Mutex mutex_;
    // Block on this until the loop starts.
    aos::Condition loop_running_cond_{&mutex_};
    // Used to notify watchers that the loop is done.
    std::atomic<bool> loop_running_{false};
    bool loop_finished_ = false;
    int priority_ = -1;
  };

  // Exclude multiple of the same type for path.

  std::vector<std::function<void()>> on_run_;
  std::shared_ptr<ThreadState> thread_state_;

  void Take(const std::string &path);

  std::vector<::std::string> taken_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
