#include <unordered_set>
#include <vector>
#include "aos/common/condition.h"
#include "aos/common/mutex.h"
#include "aos/events/event-loop.h"

namespace aos {
namespace internal {

class WatcherThreadState;
class TimerHandlerState;

}  // namespace internal

// Specialization of EventLoop that is build from queues running out of shared
// memory. See more details at aos/common/queue.h
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
  };

  // Exclude multiple of the same type for path.
  void Take(const std::string &path);

  std::vector<std::function<void()>> on_run_;
  std::shared_ptr<ThreadState> thread_state_;

  std::unordered_set<std::string> taken_;
};

}  // namespace aos
