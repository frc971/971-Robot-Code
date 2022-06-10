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
  CHECK_GE(start, monotonic_clock::epoch());
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

void EPoll::BeforeWait(std::function<void()> function) {
  before_epoll_wait_functions_.emplace_back(std::move(function));
}

void EPoll::Run() {
  run_ = true;
  // As long as run_ is true or we still have events to process, keep polling.
  while (true) {
    // If we ran out of events and Quit() was called, quit
    if (!Poll(run_)) {
      if (!run_) {
        return;
      }
    }
  }
}

bool EPoll::Poll(bool block) {
  for (const std::function<void()> &function : before_epoll_wait_functions_) {
    function();
  }
  // Pull a single event out.  Infinite timeout if we are supposed to be
  // running, and 0 length timeout otherwise.  This lets us flush the event
  // queue before quitting.
  struct epoll_event event;
  int num_events = epoll_wait(epoll_fd_, &event, 1, block ? -1 : 0);
  // Retry on EINTR and nothing else.
  if (num_events == -1) {
    if (errno == EINTR) {
      return false;
    }
    PCHECK(num_events != -1);
  }

  if (num_events == 0) {
    return false;
  }

  EventData *const event_data = static_cast<struct EventData *>(event.data.ptr);
  event_data->DoCallbacks(event.events);
  return true;
}

void EPoll::Quit() {
  // Shortcut to break us out of infinite loops. We might write more than once
  // to the pipe, but we'll stop once the first is read on the other end.
  if (!run_) {
    return;
  }
  PCHECK(write(quit_signal_fd_, "q", 1) == 1);
}

void EPoll::OnReadable(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    CHECK(!static_cast<InOutEventData *>(event_data)->in_fn)
        << ": Duplicate in functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->in_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kInEvents);
}

void EPoll::OnError(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    CHECK(!static_cast<InOutEventData *>(event_data)->err_fn)
        << ": Duplicate error functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->err_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kErrorEvents);
}

void EPoll::OnWriteable(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    CHECK(!static_cast<InOutEventData *>(event_data)->out_fn)
        << ": Duplicate out functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->out_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kOutEvents);
}

void EPoll::OnEvents(int fd, ::std::function<void(uint32_t)> function) {
  if (GetEventData(fd) != nullptr) {
    LOG(FATAL) << "May not replace OnEvents handlers";
  }
  fns_.emplace_back(std::make_unique<SingleEventData>(fd));
  static_cast<SingleEventData *>(fns_.back().get())->fn = std::move(function);
}

void EPoll::ForgetClosedFd(int fd) {
  auto element = fns_.begin();
  while (fns_.end() != element) {
    if (element->get()->fd == fd) {
      fns_.erase(element);
      return;
    }
    ++element;
  }
  LOG(FATAL) << "fd " << fd << " not found";
}

void EPoll::SetEvents(int fd, uint32_t events) {
  DoEpollCtl(CHECK_NOTNULL(GetEventData(fd)), events);
}

// Removes fd from the event loop.
void EPoll::DeleteFd(int fd) {
  auto element = fns_.begin();
  while (fns_.end() != element) {
    if (element->get()->fd == fd) {
      fns_.erase(element);
      PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) == 0)
          << "Failed to delete fd " << fd;
      return;
    }
    ++element;
  }
  LOG(FATAL) << "fd " << fd << " not found";
}

void EPoll::InOutEventData::DoCallbacks(uint32_t events) {
  if (events & kInEvents) {
    CHECK(in_fn) << ": No handler registered for input events on " << fd;
    in_fn();
  }
  if (events & kOutEvents) {
    CHECK(out_fn) << ": No handler registered for output events on " << fd;
    out_fn();
  }
  if (events & kErrorEvents) {
    CHECK(err_fn) << ": No handler registered for error events on " << fd;
    err_fn();
  }
}

void EPoll::EnableEvents(int fd, uint32_t events) {
  EventData *const event_data = CHECK_NOTNULL(GetEventData(fd));
  DoEpollCtl(event_data, event_data->events | events);
}

void EPoll::DisableEvents(int fd, uint32_t events) {
  EventData *const event_data = CHECK_NOTNULL(GetEventData(fd));
  DoEpollCtl(event_data, event_data->events & ~events);
}

EPoll::EventData *EPoll::GetEventData(int fd) {
  const auto iterator = std::find_if(
      fns_.begin(), fns_.end(),
      [fd](const std::unique_ptr<EventData> &data) { return data->fd == fd; });
  if (iterator == fns_.end()) {
    return nullptr;
  }
  return iterator->get();
}

void EPoll::DoEpollCtl(EventData *event_data, const uint32_t new_events) {
  const uint32_t old_events = event_data->events;
  if (old_events == new_events) {
    // Shortcut without calling into the kernel. This happens often with
    // external event loop integrations that are emulating poll, so make it
    // fast.
    return;
  }
  event_data->events = new_events;
  if (new_events == 0) {
    // It was added, but should now be removed.
    PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, event_data->fd, nullptr) == 0);
    return;
  }

  int operation = EPOLL_CTL_MOD;
  if (old_events == 0) {
    // If it wasn't added before, then this is the first time it's being added.
    operation = EPOLL_CTL_ADD;
  }
  struct epoll_event event;
  event.events = event_data->events;
  event.data.ptr = event_data;
  PCHECK(epoll_ctl(epoll_fd_, operation, event_data->fd, &event) == 0)
      << ": Failed to " << operation << " epoll fd: " << event_data->fd;
}

}  // namespace internal
}  // namespace aos
