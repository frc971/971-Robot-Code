#include "aos/mutex/mutex.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "aos/type_traits/type_traits.h"
#include "glog/logging.h"

namespace aos {

// Lock and Unlock use the return values of mutex_lock/mutex_unlock
// to determine whether the lock/unlock succeeded.

bool Mutex::Lock() {
  const int ret = mutex_grab(&impl_);
  if (ret == 0) {
    return false;
  } else if (ret == 1) {
    return true;
  } else {
    LOG(FATAL) << "mutex_grab(" << &impl_ << "(=" << std::hex << impl_.futex
               << ")) failed with " << ret;
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
      LOG(FATAL) << "mutex_trylock(" << &impl_ << "(=" << std::hex
                 << impl_.futex << ")) failed with " << ret;
  }
}

bool Mutex::OwnedBySelf() const {
  return mutex_islocked(&impl_);
}

}  // namespace aos
