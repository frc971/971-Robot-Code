#include "aos/common/event.h"

#include "aos/common/type_traits.h"
#include "aos/common/logging/logging.h"

namespace aos {

Event::Event() : impl_(0) {
  static_assert(shm_ok<Event>::value,
                "Event is not safe for use in shared memory.");
}

void Event::Wait() {
  int ret;
  do {
    ret = futex_wait(&impl_);
  } while (ret == 1);
  if (ret == 0) return;
  CHECK_EQ(-1, ret);
  PLOG(FATAL, "futex_wait(%p) failed", &impl_);
}

bool Event::WaitTimeout(const ::aos::time::Time &timeout) {
  const auto timeout_timespec = timeout.ToTimespec();
  int ret;
  do {
    ret = futex_wait_timeout(&impl_, &timeout_timespec);
  } while (ret == 1);
  if (ret == 0) return true;
  if (ret == 2) return false;
  CHECK_EQ(-1, ret);
  PLOG(FATAL, "futex_wait(%p) failed", &impl_);
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
