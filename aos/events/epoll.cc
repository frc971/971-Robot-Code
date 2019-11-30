#include "aos/events/epoll.h"

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <atomic>
#include <vector>

#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos {
namespace internal {

TimerFd::TimerFd()
    : fd_(timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK)) {
  PCHECK(fd_ != -1);
  Disable();
}

TimerFd::~TimerFd() { PCHECK(close(fd_) == 0); }

void TimerFd::SetTime(monotonic_clock::time_point start,
                      monotonic_clock::duration interval) {
  struct itimerspec new_value;
  new_value.it_interval = ::aos::time::to_timespec(interval);
  new_value.it_value = ::aos::time::to_timespec(start);
  PCHECK(timerfd_settime(fd_, TFD_TIMER_ABSTIME, &new_value, nullptr) == 0);
}

uint64_t TimerFd::Read() {
  uint64_t buf;
  ssize_t result = read(fd_, &buf, sizeof(buf));
  if (result == -1) {
    if (errno == EAGAIN) {
      return 0;
    }
  }
  PCHECK(result != -1);
  CHECK_EQ(result, static_cast<int>(sizeof(buf)));

  return buf;
}

EPoll::EPoll() : epoll_fd_(epoll_create1(EPOLL_CLOEXEC)) {
  PCHECK(epoll_fd_ > 0);

  // Create a pipe for the Quit function.  We want to use a pipe to be async
  // safe so this can be called from signal handlers.
  int pipefd[2];
  PCHECK(pipe2(pipefd, O_CLOEXEC | O_NONBLOCK) == 0);
  quit_epoll_fd_ = pipefd[0];
  quit_signal_fd_ = pipefd[1];
  // Read the fd when data is sent and set run_ to false.
  OnReadable(quit_epoll_fd_, [this]() {
    run_ = false;
    char buf[1];
    PCHECK(read(quit_epoll_fd_, &buf[0], 1) == 1);
  });
}

EPoll::~EPoll() {
  // Clean up the quit pipe and epoll fd.
  DeleteFd(quit_epoll_fd_);
  close(quit_signal_fd_);
  close(quit_epoll_fd_);
  CHECK_EQ(fns_.size(), 0u)
      << ": Not all file descriptors were unregistered before shutting down.";
  close(epoll_fd_);
}

void EPoll::Run() {
  while (true) {
    // Pull a single event out.  Infinite timeout if we are supposed to be
    // running, and 0 length timeout otherwise.  This lets us flush the event
    // queue before quitting.
    struct epoll_event event;
    int num_events = epoll_wait(epoll_fd_, &event, 1, run_ ? -1 : 0);
    // Retry on EINTR and nothing else.
    if (num_events == -1) {
      if (errno == EINTR) {
        continue;
      }
      PCHECK(num_events != -1);
    }
    if (!run_) {
      // If we ran out of events, quit.
      if (num_events == 0) {
        return;
      }
    }
    EventData *event_data = static_cast<struct EventData *>(event.data.ptr);
    if (event.events & (EPOLLIN | EPOLLPRI)) {
      event_data->in_fn();
    }
  }
}

void EPoll::Quit() { PCHECK(write(quit_signal_fd_, "q", 1) == 1); }

void EPoll::OnReadable(int fd, ::std::function<void()> function) {
  ::std::unique_ptr<EventData> event_data(
      new EventData{fd, ::std::move(function)});

  struct epoll_event event;
  event.events = EPOLLIN | EPOLLPRI;
  event.data.ptr = event_data.get();
  PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &event) == 0)
      << ": Failed to add fd " << fd;
  fns_.push_back(::std::move(event_data));
}

// Removes fd from the event loop.
void EPoll::DeleteFd(int fd) {
  auto element = fns_.begin();
  PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) == 0);
  while (fns_.end() != element) {
    if (element->get()->fd == fd) {
      fns_.erase(element);
      return;
    }
    ++element;
  }
  LOG(FATAL) << "fd " << fd << " not found";
}

}  // namespace internal
}  // namespace aos
