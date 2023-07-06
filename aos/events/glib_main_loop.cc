#include "aos/events/glib_main_loop.h"

#include "glog/logging.h"

namespace aos {
namespace {

gint EpollToGio(uint32_t epoll) {
  gint result = 0;
  if (epoll & EPOLLIN) {
    result |= G_IO_IN;
    epoll &= ~EPOLLIN;
  }
  if (epoll & EPOLLOUT) {
    result |= G_IO_OUT;
    epoll &= ~EPOLLOUT;
  }
  if (epoll & (EPOLLRDHUP | EPOLLHUP)) {
    result |= G_IO_HUP;
    epoll &= ~(EPOLLRDHUP | EPOLLHUP);
  }
  if (epoll & EPOLLPRI) {
    result |= G_IO_PRI;
    epoll &= ~EPOLLPRI;
  }
  if (epoll & EPOLLERR) {
    result |= G_IO_ERR;
    epoll &= ~EPOLLERR;
  }
  CHECK_EQ(epoll, 0u) << ": Unhandled epoll bits";
  return result;
}

uint32_t GioToEpoll(gint gio) {
  uint32_t result = 0;
  if (gio & G_IO_IN) {
    result |= EPOLLIN;
    gio &= ~G_IO_IN;
  }
  if (gio & G_IO_OUT) {
    result |= EPOLLOUT;
    gio &= ~G_IO_OUT;
  }
  if (gio & G_IO_HUP) {
    result |= EPOLLHUP;
    gio &= ~G_IO_HUP;
  }
  if (gio & G_IO_PRI) {
    result |= EPOLLPRI;
    gio &= ~G_IO_PRI;
  }
  if (gio & G_IO_ERR) {
    result |= EPOLLERR;
    gio &= ~G_IO_ERR;
  }
  CHECK_EQ(gio, 0) << ": Unhandled gio bits";
  return result;
}

}  // namespace

GlibMainLoop::GlibMainLoop(ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      timeout_timer_(event_loop->AddTimer([]() {
        // Don't need to do anything, just need to get the event loop to break
        // out of the kernel and call BeforeWait again.
      })),
      g_main_context_(g_main_context_ref(g_main_context_default())),
      g_main_loop_(g_main_loop_new(g_main_context_, true)) {
  event_loop_->OnRun([this]() {
    CHECK(!acquired_context_);
    CHECK(g_main_context_acquire(g_main_context_))
        << ": The EventLoop thread must own the context";
    acquired_context_ = true;
  });
  event_loop_->epoll()->BeforeWait([this]() { BeforeWait(); });
}

GlibMainLoop::~GlibMainLoop() {
  CHECK_EQ(children_, 0) << ": Failed to destroy all children";
  RemoveAllFds();
  if (acquired_context_) {
    g_main_context_release(g_main_context_);
  }
  g_main_loop_unref(g_main_loop_);
  g_main_context_unref(g_main_context_);
}

void GlibMainLoop::RemoveAllFds() {
  while (true) {
    const auto to_remove = added_fds_.begin();
    if (to_remove == added_fds_.end()) {
      break;
    }
    event_loop_->epoll()->DeleteFd(*to_remove);
    added_fds_.erase(to_remove);
  }
}

void GlibMainLoop::BeforeWait() {
  if (!g_main_loop_is_running(g_main_loop_)) {
    // glib will never quiesce its FDs, so the best we can do is just skip it
    // once it's done and shut down our event loop. We have to remove all of its
    // FDs first so other event sources can quiesce.
    VLOG(1) << "g_main_loop_is_running = false";
    RemoveAllFds();
    event_loop_->Exit();
    return;
  }
  if (!event_loop_->epoll()->should_run()) {
    // Give glib one more round of dispatching.
    VLOG(1) << "EPoll::should_run = false";
    g_main_loop_quit(g_main_loop_);
  }

  if (!gpoll_fds_.empty()) {
    // Tell glib about any events we received on the FDs it asked about.
    if (g_main_context_check(g_main_context_, last_query_max_priority_,
                             gpoll_fds_.data(), gpoll_fds_.size())) {
      VLOG(1) << "g_main_context_dispatch";
      // We have some glib events now, dispatch them now.
      g_main_context_dispatch(g_main_context_);
    }
  }

  // Call prepare to check for any other events that are ready to be dispatched.
  // g_main_context_iterate ignores the return value, so we're going to do that
  // too.
  g_main_context_prepare(g_main_context_, &last_query_max_priority_);

  gint timeout_ms;
  while (true) {
    const gint number_new_fds =
        g_main_context_query(g_main_context_, last_query_max_priority_,
                             &timeout_ms, gpoll_fds_.data(), gpoll_fds_.size());
    if (static_cast<size_t>(number_new_fds) <= gpoll_fds_.size()) {
      // They all fit, resize to drop any stale entries and then we're done.
      gpoll_fds_.resize(number_new_fds);
      VLOG(1) << "glib gave " << number_new_fds;
      break;
    }
    // Need more space, we know how much so try again.
    gpoll_fds_.resize(number_new_fds);
  }

  for (GPollFD gpoll_fd : gpoll_fds_) {
    // API docs are a bit unclear, but this shouldn't ever happen I think?
    CHECK_EQ(gpoll_fd.revents, 0) << ": what does this mean?";

    if (added_fds_.count(gpoll_fd.fd) == 0) {
      VLOG(1) << "Add to ShmEventLoop: " << gpoll_fd.fd;
      event_loop_->epoll()->OnEvents(
          gpoll_fd.fd, [this, fd = gpoll_fd.fd](uint32_t events) {
            VLOG(1) << "glib " << fd << " triggered: " << std::hex << events;
            const auto iterator = std::find_if(
                gpoll_fds_.begin(), gpoll_fds_.end(),
                [fd](const GPollFD &candidate) { return candidate.fd == fd; });
            CHECK(iterator != gpoll_fds_.end())
                << ": Lost GPollFD for " << fd
                << " but still registered with epoll";
            iterator->revents |= EpollToGio(events);
          });
      added_fds_.insert(gpoll_fd.fd);
    }
    event_loop_->epoll()->SetEvents(gpoll_fd.fd, GioToEpoll(gpoll_fd.events));
  }
  for (int fd : added_fds_) {
    const auto iterator = std::find_if(
        gpoll_fds_.begin(), gpoll_fds_.end(),
        [fd](const GPollFD &candidate) { return candidate.fd == fd; });
    if (iterator == gpoll_fds_.end()) {
      VLOG(1) << "Remove from ShmEventLoop: " << fd;
      added_fds_.erase(fd);
    }
  }
  CHECK_EQ(added_fds_.size(), gpoll_fds_.size());
  VLOG(1) << "Timeout: " << timeout_ms;
  if (timeout_ms == -1) {
    timeout_timer_->Disable();
  } else {
    timeout_timer_->Schedule(event_loop_->monotonic_now() +
                             std::chrono::milliseconds(timeout_ms));
  }
}

}  // namespace aos
