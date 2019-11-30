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

  // Quits.  Async safe.
  void Quit();

  // Registers a function to be called if the fd becomes readable.
  // There should only be 1 function registered for each fd.
  void OnReadable(int fd, ::std::function<void()> function);

  // Removes fd from the event loop.
  // All Fds must be cleaned up before this class is destroyed.
  void DeleteFd(int fd);

 private:
  ::std::atomic<bool> run_{true};

  // Main epoll fd.
  int epoll_fd_;

  // Structure whose pointer should be returned by epoll.  Makes looking up the
  // function fast and easy.
  struct EventData {
    int fd;
    ::std::function<void()> in_fn;
  };
  ::std::vector<::std::unique_ptr<EventData>> fns_;

  // Pipe pair for handling quit.
  int quit_signal_fd_;
  int quit_epoll_fd_;
};

}  // namespace internal
}  // namespace aos

#endif  // AOS_EVENTS_EPOLL_H_
