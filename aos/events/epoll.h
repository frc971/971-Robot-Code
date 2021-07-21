#ifndef AOS_EVENTS_EPOLL_H_
#define AOS_EVENTS_EPOLL_H_

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <atomic>
#include <functional>
#include <vector>

#include "aos/time/time.h"

namespace aos {
namespace internal {

// Class wrapping up timerfd.
class TimerFd {
 public:
  TimerFd();
  ~TimerFd();

  TimerFd(const TimerFd &) = delete;
  TimerFd &operator=(const TimerFd &) = delete;
  TimerFd(TimerFd &&) = delete;
  TimerFd &operator=(TimerFd &&) = delete;

  // Sets the trigger time and repeat for the timerfd.
  // An interval of 0 results in a single expiration.
  void SetTime(monotonic_clock::time_point start,
               monotonic_clock::duration interval);

  // Disarms the timer.
  void Disable() {
    // Disarm the timer by feeding zero values
    SetTime(::aos::monotonic_clock::epoch(), ::aos::monotonic_clock::zero());
  }

  // Reads the event.  Returns the number of elapsed cycles.
  uint64_t Read();

  // Returns the file descriptor associated with the timerfd.
  int fd() { return fd_; }

 private:
  int fd_ = -1;
};

// Class to wrap epoll and call a callback when an event happens.
class EPoll {
 public:
  EPoll();
  ~EPoll();
  EPoll(const EPoll &) = delete;
  EPoll &operator=(const EPoll &) = delete;
  EPoll(EPoll &&) = delete;
  EPoll &operator=(EPoll &&) = delete;

  // Runs until Quit() is called.
  void Run();

  // Consumes a single epoll event. Blocks indefinitely if block is true, or
  // does not block at all. Returns true if an event was consumed, and false on
  // any retryable error or if no events are available. Dies fatally on
  // non-retryable errors.
  bool Poll(bool block);

  // Quits.  Async safe.
  void Quit();

  // Adds a function which will be called before waiting on the epoll file
  // descriptor.
  void BeforeWait(std::function<void()> function);

  // Registers a function to be called if the fd becomes readable.
  // Only one function may be registered for readability on each fd.
  // A fd may be registered exclusively with OnReadable/OnWriteable/OnError OR
  // OnEvents.
  void OnReadable(int fd, ::std::function<void()> function);

  // Registers a function to be called if the fd reports an error.
  // Only one function may be registered for errors on each fd.
  // A fd may be registered exclusively with OnReadable/OnWriteable/OnError OR
  // OnEvents.
  void OnError(int fd, ::std::function<void()> function);

  // Registers a function to be called if the fd becomes writeable.
  // Only one function may be registered for writability on each fd.
  // A fd may be registered exclusively with OnReadable/OnWriteable/OnError OR
  // OnEvents.
  void OnWriteable(int fd, ::std::function<void()> function);

  // Registers a function to be called when the configured events occur on fd.
  // Which events occur will be passed to the function.
  // A fd may be registered exclusively with OnReadable/OnWriteable/OnError OR
  // OnEvents.
  void OnEvents(int fd, ::std::function<void(uint32_t)> function);

  // Removes fd from the event loop.
  // All Fds must be cleaned up before this class is destroyed.
  void DeleteFd(int fd);

  // Removes a closed fd.  When fds are closed, they are automatically
  // unregistered by the kernel.  But we need to clean up any state here.
  // All Fds must be cleaned up before this class is destroyed.
  void ForgetClosedFd(int fd);

  // Enables calling the existing function registered for fd when it becomes
  // writeable.
  void EnableWriteable(int fd) { EnableEvents(fd, kOutEvents); }

  // Disables calling the existing function registered for fd when it becomes
  // writeable.
  void DisableWriteable(int fd) { DisableEvents(fd, kOutEvents); }

  // Sets the epoll events for the given fd. Be careful using this with
  // OnReadable/OnWriteable/OnError: enabled events which fire with no handler
  // registered will result in a crash.
  void SetEvents(int fd, uint32_t events);

  // Returns whether we're currently running. This changes to false when we
  // start draining events to finish.
  bool should_run() const { return run_; }

 private:
  // Structure whose pointer should be returned by epoll.  Makes looking up the
  // function fast and easy.
  struct EventData {
    EventData(int fd_in) : fd(fd_in) {}
    virtual ~EventData() = default;

    // We use pointers to these objects as persistent identifiers, so they can't
    // be moved.
    EventData(const EventData &) = delete;
    EventData &operator=(const EventData &) = delete;

    // Calls the appropriate callbacks when events are returned from the kernel.
    virtual void DoCallbacks(uint32_t events) = 0;

    const int fd;
    uint32_t events = 0;
  };

  struct InOutEventData : public EventData {
    InOutEventData(int fd) : EventData(fd) {}
    ~InOutEventData() override = default;

    std::function<void()> in_fn, out_fn, err_fn;

    void DoCallbacks(uint32_t events) override;
  };

  struct SingleEventData : public EventData {
    SingleEventData(int fd) : EventData(fd) {}
    ~SingleEventData() override = default;

    std::function<void(uint32_t)> fn;

    void DoCallbacks(uint32_t events) override { fn(events); }
  };

  void EnableEvents(int fd, uint32_t events);
  void DisableEvents(int fd, uint32_t events);

  EventData *GetEventData(int fd);

  void DoEpollCtl(EventData *event_data, uint32_t new_events);

  // TODO(Brian): Figure out a nicer way to handle EPOLLPRI than lumping it in
  // with input.
  static constexpr uint32_t kInEvents = EPOLLIN | EPOLLPRI;
  static constexpr uint32_t kOutEvents = EPOLLOUT;
  static constexpr uint32_t kErrorEvents = EPOLLERR;

  ::std::atomic<bool> run_{true};

  // Main epoll fd.
  int epoll_fd_;

  ::std::vector<::std::unique_ptr<EventData>> fns_;

  // Pipe pair for handling quit.
  int quit_signal_fd_;
  int quit_epoll_fd_;

  std::vector<std::function<void()>> before_epoll_wait_functions_;
};

}  // namespace internal
}  // namespace aos

#endif  // AOS_EVENTS_EPOLL_H_
