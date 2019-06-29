#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <unordered_set>
#include <vector>

#include "aos/condition.h"
#include "aos/events/epoll.h"
#include "aos/events/event-loop.h"
#include "aos/mutex/mutex.h"

namespace aos {
namespace internal {

class WatcherThreadState;
class TimerHandlerState;
class PhasedLoopHandler;

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

  ::std::unique_ptr<RawSender> MakeRawSender(
      const ::std::string &path, const QueueTypeInfo &type) override;
  ::std::unique_ptr<RawFetcher> MakeRawFetcher(
      const ::std::string &path, const QueueTypeInfo &type) override;

  void MakeRawWatcher(
      const ::std::string &path, const QueueTypeInfo &type,
      ::std::function<void(const aos::Message *message)> watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override;
  ::aos::PhasedLoopHandler *AddPhasedLoop(
      ::std::function<void(int)> callback,
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset =
          ::std::chrono::seconds(0)) override;

  void OnRun(::std::function<void()> on_run) override;
  void Run() override;
  void Exit() override;

  // TODO(austin): Add a function to register control-C call.

  void SetRuntimeRealtimePriority(int priority) override {
    if (is_running()) {
      ::aos::Die("Cannot set realtime priority while running.");
    }
    thread_state_.priority_ = priority;
  }

  void set_name(const char *name) override;

 private:
  friend class internal::WatcherThreadState;
  friend class internal::TimerHandlerState;
  friend class internal::PhasedLoopHandler;
  // This ThreadState ensures that two watchers in the same loop cannot be
  // triggered concurrently.  Because watchers block threads indefinitely, this
  // has to be shared_ptr in case the EventLoop is destroyed before the thread
  // receives any new events.
  class ThreadState {
   public:
    void WaitForStart();

    bool is_running() { return loop_running_; }

    void Start();

    void Exit();

    void MaybeSetCurrentThreadRealtimePriority();

    const ::std::string &name() const { return name_; }

   private:
    friend class internal::WatcherThreadState;
    friend class internal::TimerHandlerState;
    friend class internal::PhasedLoopHandler;
    friend class ShmEventLoop;

    // This mutex ensures that only one watch event happens at a time.
    ::aos::Mutex mutex_;
    // Block on this until the loop starts.
    ::aos::Condition loop_running_cond_{&mutex_};
    // Used to notify watchers that the loop is done.
    ::std::atomic<bool> loop_running_{false};
    bool loop_finished_ = false;
    int priority_ = -1;

    // Immutable after Start is called.
    ::std::string name_;
  };

  // Tracks that we can't have multiple watchers or a sender and a watcher (or
  // multiple senders) on a single queue (path).
  void Take(const ::std::string &path);

  ::std::vector<::std::function<void()>> on_run_;
  ThreadState thread_state_;
  ::std::vector<::std::string> taken_;
  internal::EPoll epoll_;

  ::std::vector<::std::unique_ptr<internal::TimerHandlerState>> timers_;
  ::std::vector<::std::unique_ptr<internal::PhasedLoopHandler>> phased_loops_;
  ::std::vector<::std::unique_ptr<internal::WatcherThreadState>> watchers_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
