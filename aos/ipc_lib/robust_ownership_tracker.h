#ifndef AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_
#define AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_

#include <linux/futex.h>

#include <string>

#include "aos/ipc_lib/aos_sync.h"

namespace aos::ipc_lib {

// Results of atomically loading the ownership state via RobustOwnershipTracker
// below. This allows the state to be compared and queried later.
class ThreadOwnerStatusSnapshot {
 public:
  ThreadOwnerStatusSnapshot() : futex_(0) {}
  ThreadOwnerStatusSnapshot(aos_futex futex) : futex_(futex) {}
  ThreadOwnerStatusSnapshot(const ThreadOwnerStatusSnapshot &) = default;
  ThreadOwnerStatusSnapshot &operator=(const ThreadOwnerStatusSnapshot &) =
      default;
  ThreadOwnerStatusSnapshot(ThreadOwnerStatusSnapshot &&) = default;
  ThreadOwnerStatusSnapshot &operator=(ThreadOwnerStatusSnapshot &&) = default;

  // Returns if the owner died as noticed by the robust futex using Acquire
  // memory ordering.
  bool OwnerIsDead() const { return (futex_ & FUTEX_OWNER_DIED) != 0; }

  // Returns true if no one has claimed ownership.
  bool IsUnclaimed() const { return futex_ == 0; }

  // Returns true if either ownership hasn't been acquired or the owner died.
  bool IsUnclaimedOrOwnerIsDead() const {
    return IsUnclaimed() || OwnerIsDead();
  }

  // Returns the thread ID (a.k.a. "tid") of the owning thread. Use this when
  // trying to access the /proc entry that corresponds to the owning thread for
  // example. Do not use the futex value directly.
  pid_t tid() const { return futex_ & FUTEX_TID_MASK; }

  bool operator==(const ThreadOwnerStatusSnapshot &other) const {
    return other.futex_ == futex_;
  }

 private:
  aos_futex futex_;
};

// This object reliably tracks a thread owning a resource. A single thread may
// possess multiple resources like senders and receivers. Each resource can have
// its own instance of this class. These instances are responsible for
// monitoring the thread that owns them. Each resource can use its instance of
// this class to reliably check whether the owning thread is no longer alive.
//
// All methods other than Load* must be accessed under a mutex.
class RobustOwnershipTracker {
 public:
  // Loads all the contents of the ownership tracker with Acquire memory
  // ordering.
  ThreadOwnerStatusSnapshot LoadAcquire() const {
    return ThreadOwnerStatusSnapshot(
        __atomic_load_n(&(mutex_.futex), __ATOMIC_ACQUIRE));
  }

  // Loads all the contents of the ownership tracker with Relaxed memory order.
  ThreadOwnerStatusSnapshot LoadRelaxed() const {
    return ThreadOwnerStatusSnapshot(
        __atomic_load_n(&(mutex_.futex), __ATOMIC_RELAXED));
  }

  // Clears all ownership state.
  //
  // This should only really be called if you are 100% certain that the owner is
  // dead. Use `LoadAquire().OwnerIsDead()` to determine this.
  void ForceClear() {
    // Must be opposite order of Acquire.
    // We only deal with the futex here because we don't want to change anything
    // about the linked list. We just want to release ownership here. We still
    // want the kernel to know about this element via the linked list the next
    // time someone takes ownership.
    __atomic_store_n(&(mutex_.futex), 0, __ATOMIC_RELEASE);
  }

  // Returns true if this thread holds ownership.
  bool IsHeldBySelf() { return death_notification_is_held(&mutex_); }

  // Acquires ownership. Other threads will know that this thread holds the
  // ownership or be notified if this thread dies.
  void Acquire() { death_notification_init(&mutex_); }

  // Releases ownership.
  //
  // This should only be called from the owning thread.
  void Release() {
    // Must be opposite order of Acquire.
    death_notification_release(&mutex_);
  }

  // Marks the owner as dead if the specified tid is the current owner. In other
  // words, after this call, a call to `LoadAcquire().OwnerIsDead()` may start
  // returning true.
  //
  // The motivation here is for use in testing. DO NOT USE in production code.
  // The logic here is only good enough for testing.
  bool PretendThatOwnerIsDeadForTesting(pid_t tid);

  // Returns a string representing this object.
  std::string DebugString() const;

 private:
  // Robust futex to track ownership the normal way. The futex is inside the
  // mutex here. We use the wrapper mutex because the death_notification_*
  // functions operate on that instead of the futex directly.
  //
  // We use a futex here because:
  // - futexes are fast.
  // - The kernel can atomically clean up a dead thread and mark the futex
  //   appropriately.
  // - Owners can clean up after dead threads.
  aos_mutex mutex_;
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_
