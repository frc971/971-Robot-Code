#include "aos/common/mutex.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "aos/common/type_traits.h"
#include "aos/common/logging/logging.h"

namespace aos {

Mutex::Mutex() : impl_() {
  static_assert(shm_ok<Mutex>::value,
                "Mutex is not safe for use in shared memory.");
}

Mutex::~Mutex() {
  if (__builtin_expect(mutex_islocked(&impl_), false)) {
    LOG(FATAL, "destroying locked mutex %p (aka %p)\n",
        this, &impl_);
  }
}

// Lock and Unlock use the return values of mutex_lock/mutex_unlock
// to determine whether the lock/unlock succeeded.

bool Mutex::Lock() {
  const int ret = mutex_grab(&impl_);
  if (ret == 0) {
    return false;
  } else if (ret == 1) {
    return true;
  } else {
    LOG(FATAL, "mutex_grab(%p(=%" PRIu32 ")) failed with %d\n",
        &impl_, impl_.futex, ret);
  }
}

void Mutex::Unlock() {
  mutex_unlock(&impl_);
}

Mutex::State Mutex::TryLock() {
  const int ret = mutex_trylock(&impl_);
  switch (ret) {
    case 0:
      return State::kLocked;
    case 1:
      return State::kOwnerDied;
    case 4:
      return State::kLockFailed;
    default:
      LOG(FATAL, "mutex_trylock(%p(=%" PRIu32 ")) failed with %d\n",
          &impl_, impl_.futex, ret);
  }
}

bool Mutex::OwnedBySelf() const {
  return mutex_islocked(&impl_);
}

}  // namespace aos
