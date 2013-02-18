#include "aos/common/mutex.h"

#include <semLib.h>
#include <string.h>

#include "aos/common/logging/logging.h"

namespace aos {

Mutex::Mutex() : impl_(semBCreate(SEM_Q_PRIORITY, SEM_FULL)) {
  if (impl_ == NULL) {
    LOG(FATAL, 
        "semBCreate(SEM_Q_PRIORITY, SEM_FULL) failed because of %d: %s\n",
        errno, strerror(errno)); 
  }
}

Mutex::~Mutex() {
  if (semDelete(impl_) != 0) {
    LOG(FATAL, "semDelete(%p) failed because of %d: %s\n",
        impl_, errno, strerror(errno));
  }
}

void Mutex::Lock() {
  if (semTake(impl_, WAIT_FOREVER) != 0) {
    LOG(FATAL, "semTake(%p, WAIT_FOREVER) failed because of %d: %s\n",
        impl_, errno, strerror(errno));
  }
}

void Mutex::Unlock() {
  if (semGive(impl_) != 0) {
    LOG(FATAL, "semGive(%p) failed because of %d: %s\n",
        impl_, errno, strerror(errno));
  }
}

bool Mutex::TryLock() {
  if (semTake(impl_, NO_WAIT) == 0) {
    return true;
  }
  // The semLib documention is wrong about what the errno will be.
  if (errno != S_objLib_OBJ_UNAVAILABLE) {
    LOG(FATAL, "semTake(%p, WAIT_FOREVER) failed because of %d: %s\n",
        impl_, errno, strerror(errno));
  }
  return false;
}

}  // namespace aos
