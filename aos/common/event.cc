#include "aos/common/event.h"

#include <chrono>

#include "aos/common/type_traits.h"
#include "aos/common/logging/logging.h"

namespace aos {

Event::Event() : impl_(0) {
  static_assert(shm_ok<Event>::value,
                "Event is not safe for use in shared memory.");
}

void Event::Wait() {
  while (__atomic_load_n(&impl_, __ATOMIC_SEQ_CST) == 0) {
    const int ret = futex_wait(&impl_);
    if (ret != 0) {
      CHECK_EQ(-1, ret);
      PLOG(FATAL, "futex_wait(%p) failed", &impl_);
    }
  }
}

bool Event::WaitTimeout(monotonic_clock::duration timeout) {
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(timeout);
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(timeout - sec);
  struct timespec timeout_timespec;
  timeout_timespec.tv_sec = sec.count();
  timeout_timespec.tv_nsec = nsec.count();
  while (true) {
    if (__atomic_load_n(&impl_, __ATOMIC_SEQ_CST) != 0) {
      return true;
    }
    const int ret = futex_wait_timeout(&impl_, &timeout_timespec);
    if (ret != 0) {
      if (ret == 2) return false;
      CHECK_EQ(-1, ret);
      PLOG(FATAL, "futex_wait(%p) failed", &impl_);
    }
  }
}

// We're not going to expose the number woken because that's not easily portable
// to condition variable-based implementations.
void Event::Set() {
  if (futex_set(&impl_) == -1) {
    PLOG(FATAL, "futex_set(%p) failed", &impl_);
  }
}

bool Event::Clear() {
  return !futex_unset(&impl_);
}

}  // namespace aos
