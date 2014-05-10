#include "aos/common/mutex.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "aos/common/type_traits.h"
#include "aos/common/logging/logging.h"

namespace aos {

Mutex::Mutex() : impl_(0) {
  static_assert(shm_ok<Mutex>::value,
                "Mutex is not safe for use in shared memory.");
}

// Lock and Unlock use the return values of mutex_lock/mutex_unlock
// to determine whether the lock/unlock succeeded.

void Mutex::Lock() {
  if (mutex_grab(&impl_) != 0) {
    PLOG(FATAL, "mutex_grab(%p(=%" PRIu32 ")) failed", &impl_, impl_);
  }
}

void Mutex::Unlock() {
  mutex_unlock(&impl_);
}

bool Mutex::TryLock() {
  return mutex_trylock(&impl_) == 0;
}

}  // namespace aos
