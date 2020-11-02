#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <vector>

#include "absl/types/span.h"

#include "aos/events/epoll.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"

DECLARE_string(application_name);
DECLARE_string(shm_base);

namespace aos {
namespace shm_event_loop_internal {

class ShmWatcherState;
class ShmTimerHandler;
class ShmPhasedLoopHandler;
class ShmSender;
class SimpleShmFetcher;
class ShmFetcher;

}  // namespace shm_event_loop_internal

// Concrete implementation of EventLoop that is built from queues running out of
// shared memory.
//
// TODO(austin): Timing reports break multiple threads.  Need to add back in a
// mutex.
// This object must be interacted with from one thread, but the Senders
// and Fetchers may be used from multiple threads afterwords (as long as their
// destructors are called back in one thread again)
class ShmEventLoop : public EventLoop {
 public:
  ShmEventLoop(const Flatbuffer<Configuration> &configuration)
      : ShmEventLoop(&configuration.message()) {}
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
  void MakeRawNoArgWatcher(
      const Channel *channel,
      std::function<void(const Context &context)> watcher) override;

  TimerHandler *AddTimer(std::function<void()> callback) override;
  PhasedLoopHandler *AddPhasedLoop(std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       std::chrono::seconds(0)) override;

  void OnRun(std::function<void()> on_run) override;

  void SetRuntimeRealtimePriority(int priority) override;
  void SetRuntimeAffinity(const cpu_set_t &cpuset) override;

  void set_name(const std::string_view name) override;
  const std::string_view name() const override { return name_; }
  const Node *node() const override { return node_; }

  int priority() const override { return priority_; }

  // Returns the epoll loop used to run the event loop.
  internal::EPoll *epoll() { return &epoll_; }

  // Returns the local mapping of the shared memory used by the watcher on the
  // specified channel. A watcher must be created on this channel before calling
  // this.
  absl::Span<const char> GetWatcherSharedMemory(const Channel *channel);

  // Returns the local mapping of the shared memory used by the provided Sender.
  template <typename T>
  absl::Span<char> GetSenderSharedMemory(aos::Sender<T> *sender) const {
    return GetShmSenderSharedMemory(GetRawSender(sender));
  }

  // Returns the local mapping of the private memory used by the provided
  // Fetcher to hold messages.
  //
  // Note that this may be the entire shared memory region held by this fetcher,
  // depending on its channel's read_method.
  template <typename T>
  absl::Span<const char> GetFetcherPrivateMemory(
      aos::Fetcher<T> *fetcher) const {
    return GetShmFetcherPrivateMemory(GetRawFetcher(fetcher));
  }

  int NumberBuffers(const Channel *channel) override;

 private:
  friend class shm_event_loop_internal::ShmWatcherState;
  friend class shm_event_loop_internal::ShmTimerHandler;
  friend class shm_event_loop_internal::ShmPhasedLoopHandler;
  friend class shm_event_loop_internal::ShmSender;
  friend class shm_event_loop_internal::SimpleShmFetcher;
  friend class shm_event_loop_internal::ShmFetcher;

  using EventLoop::SendTimingReport;

  static cpu_set_t DefaultAffinity() {
    cpu_set_t result;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      CPU_SET(i, &result);
    }
    return result;
  }

  void HandleEvent();

  // Returns the TID of the event loop.
  pid_t GetTid() override;

  // Private method to access the shared memory mapping of a ShmSender.
  absl::Span<char> GetShmSenderSharedMemory(const aos::RawSender *sender) const;

  // Private method to access the private memory mapping of a ShmFetcher.
  absl::Span<const char> GetShmFetcherPrivateMemory(
      const aos::RawFetcher *fetcher) const;

  // Capture the --shm_base flag at construction time.  This makes it much
  // easier to make different shared memory regions for doing things like
  // multi-node tests.
  std::string shm_base_;

  std::vector<std::function<void()>> on_run_;
  int priority_ = 0;
  cpu_set_t affinity_ = DefaultAffinity();
  std::string name_;
  const Node *const node_;

  internal::EPoll epoll_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
