#include "aos/common/condition.h"

#include <inttypes.h>

#include "aos/common/type_traits.h"

namespace aos {

static_assert(shm_ok<Condition>::value, "Condition should work"
              " in shared memory");

Condition::Condition() : impl_(0) {}

bool Condition::Wait() {
  switch (condition_wait(&impl_)) {
    case 1:
      return false;
    case 0:
      return true;
    default:
      if (errno != EINTR) {
        LOG(FATAL, "condition_wait(%p(=%"PRIu32")) failed because of %d: %s\n",
            &impl_, impl_, errno, strerror(errno));
      }
      return false;
  }
}
bool Condition::WaitNext() {
  switch (condition_wait_force(&impl_)) {
    case 1:
      return false;
    case 0:
      return true;
    default:
      if (errno != EINTR) {
        LOG(FATAL, "condition_wait_force(%p(=%"PRIu32")) failed"
            " because of %d: %s\n", &impl_, impl_, errno, strerror(errno));
      }
      return false;
  }
}

void Condition::Set() {
  if (condition_set(&impl_) == -1) {
    LOG(FATAL, "condition_set(%p(=%"PRIu32")) failed because of %d: %s\n",
        &impl_, impl_, errno, strerror(errno));
  }
}
void Condition::Unset() {
  // can not fail
  condition_unset(&impl_);
}

}  // namespace aos
