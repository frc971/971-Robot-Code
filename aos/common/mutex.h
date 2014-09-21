#ifndef AOS_COMMON_MUTEX_H_
#define AOS_COMMON_MUTEX_H_

#ifdef __VXWORKS__
#include <semLib.h>
#endif

#include "aos/common/macros.h"
#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/aos_sync.h"

namespace aos {

class Condition;

// An abstraction of a mutex that has implementations both for
// linux and for the cRIO.
// If there are multiple tasks or processes contending for the mutex,
// higher priority ones will succeed in locking first,
// and tasks of equal priorities have the same chance of getting the lock.
// There is no priority inversion protection.
class Mutex {
 public:
  // Creates an unlocked mutex.
  Mutex();
#ifdef __VXWORKS__
  // Will not make sure that it is either locked or unlocked.
  ~Mutex();
#endif
  // Locks the mutex. If it fails, it calls LOG(FATAL).
  // Returns false.
  bool Lock();
  // Unlocks the mutex. Fails like Lock.
  // Multiple unlocking is undefined.
  void Unlock();
  // Locks the mutex unless it is already locked.
  // Returns whether it succeeded or not.
  // Doesn't wait for the mutex to be unlocked if it is locked.
  bool TryLock();

 private:
#ifdef __VXWORKS__
  typedef SEM_ID ImplementationType;
#else
  typedef mutex ImplementationType;
#endif
  ImplementationType impl_;

  friend class Condition;  // for access to impl_

#ifdef __VXWORKS__
  DISALLOW_COPY_AND_ASSIGN(Mutex);
#endif
};

// A class that locks a Mutex when constructed and unlocks it when destructed.
// Designed to be used as a local variable so that
// the mutex will be unlocked when the scope is exited.
class MutexLocker {
 public:
  explicit MutexLocker(Mutex *mutex) : mutex_(mutex) {
    mutex_->Lock();
  }
  ~MutexLocker() {
    mutex_->Unlock();
  }

 private:
  Mutex *mutex_;
  DISALLOW_COPY_AND_ASSIGN(MutexLocker);
};
// The inverse of MutexLocker.
class MutexUnlocker {
 public:
  explicit MutexUnlocker(Mutex *mutex) : mutex_(mutex) {
    mutex_->Unlock();
  }
  ~MutexUnlocker() {
    mutex_->Lock();
  }

 private:
  Mutex *mutex_;
  DISALLOW_COPY_AND_ASSIGN(MutexUnlocker);
};

}  // namespace aos

#endif  // AOS_COMMON_MUTEX_H_
