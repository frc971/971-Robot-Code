#include "aos/common/mutex.h"

#include <inttypes.h>
#include <errno.h>

#include "aos/aos_core.h"
#include "aos/common/type_traits.h"

namespace aos {

Mutex::Mutex() : impl_(0) {
  static_assert(shm_ok<Mutex>::value,
                "Mutex is not safe for use in shared memory.");
}

// Lock and Unlock use the return values of mutex_lock/mutex_unlock
// to determine whether the lock/unlock succeeded.

void Mutex::Lock() {
  if (mutex_grab(&impl_) != 0) {
    LOG(FATAL, "mutex_grab(%p(=%"PRIu32")) failed because of %d: %s\n",
        &impl_, impl_, errno, strerror(errno));
  }
}

void Mutex::Unlock() {
  if (mutex_unlock(&impl_) != 0) {
    LOG(FATAL, "mutex_unlock(%p(=%"PRIu32")) failed because of %d: %s\n",
        &impl_, impl_, errno, strerror(errno));
  }
}

bool Mutex::TryLock() {
  return mutex_trylock(&impl_) == 0;
}

}  // namespace aos
