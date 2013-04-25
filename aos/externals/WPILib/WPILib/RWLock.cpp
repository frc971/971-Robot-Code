/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include <taskLib.h>
#include <intLib.h>
#include <assert.h>

#include "RWLock.h"

// A wrapper for assert that allows it to be easily turned off just in this
// file. That configuration is recommended for normal use because it means less
// code that gets executed with the scheduler locked.
#if 1
#define rwlock_assert(expression) assert(expression)
// A macro to easily assert that some expression (possibly with side effects)
// is 0.
#define rwlock_assert_success(expression) do { \
  int ret = (expression); \
  assert(ret == 0); \
} while (false)
#else
#define rwlock_assert(expression) ((void)0)
#define rwlock_assert_success(expression) ((void)(expression))
#endif

/**
 * Class that locks the scheduler and then unlocks it in the destructor.
 */
class TaskSchedulerLocker {
 public:
  TaskSchedulerLocker() {
    rwlock_assert_success(taskLock());
  }
  ~TaskSchedulerLocker() {
    rwlock_assert_success(taskUnlock());
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(TaskSchedulerLocker);
};

RWLock::Locker::Locker(RWLock *lock, bool write) : lock_(lock) {
  lock_->Lock(write);
}

RWLock::Locker::Locker(const Locker &other) : lock_(other.lock_) {
  lock_->AddLock();
}

RWLock::Locker::~Locker() {
  lock_->Unlock();
}

// RWLock is implemented by just locking the scheduler while doing anything
// because that is the only way under vxworks to do much of anything atomically.

void RWLock::Lock(bool write) {
  assert(!intContext());
  TaskSchedulerLocker scheduler_locker;

  // We can't be reading and writing at the same time.
  rwlock_assert(!((number_of_write_locks_ > 0) && (number_of_readers_ > 0)));

  if (write) {
    // If somebody else already has it locked.
    // Don't have to worry about another task getting scheduled after
    // write_ready_ gets given because nobody else (except another writer) will
    // lock anything while there are pending writer(s).
    if ((number_of_readers_ > 0) || (number_of_write_locks_ > 0)) {
      ++number_of_writers_pending_;
      // Wait for it to be our turn.
      rwlock_assert_success(semTake(write_ready_, WAIT_FOREVER));
      --number_of_writers_pending_;
    } else {
      rwlock_assert(number_of_writers_pending_ == 0);
    }
    rwlock_assert((number_of_write_locks_ == 0) && (number_of_readers_ == 0));
    number_of_write_locks_ = 1;
  } else {  // read
    // While there are one or more writers active or waiting.
    // Has to be a loop in case a writer gets scheduled between the time
    // read_ready_ gets flushed and we run.
    while ((number_of_write_locks_ > 0) || (number_of_writers_pending_ > 0)) {
      // Wait for the writer(s) to finish.
      rwlock_assert_success(semTake(read_ready_, WAIT_FOREVER));
    }
    ++number_of_readers_;
    rwlock_assert((number_of_write_locks_ == 0) && (number_of_readers_ > 0));
  }
}

void RWLock::Unlock() {
  assert(!intContext());
  TaskSchedulerLocker scheduler_locker;

  // We have to be reading or writing right now, but not both.
  rwlock_assert((number_of_write_locks_ > 0) != (number_of_readers_ > 0));

  if (number_of_write_locks_ > 0) {  // we're currently writing
    --number_of_write_locks_;
    rwlock_assert((number_of_write_locks_ >= 0) &&
                  (number_of_writers_pending_ >= 0));
    // If we were the last one.
    if (number_of_write_locks_ == 0) {
      // If there are no other tasks waiting to write (because otherwise they
      // need to get priority over any readers).
      if (number_of_writers_pending_ == 0) {
        // Wake up any waiting readers.
        rwlock_assert_success(semFlush(read_ready_));
      } else {
        // Wake up a waiting writer.
        // Not a problem if somebody else already did this before the waiting
        // writer got a chance to take it because it'll still return success.
        rwlock_assert_success(semGive(write_ready_));
      }
    }
  } else {  // we're curently reading
    --number_of_readers_;
    rwlock_assert(number_of_readers_ >= 0 &&
                  (number_of_writers_pending_ >= 0));
    // If we were the last one.
    if (number_of_readers_ == 0) {
      // If there are any writers waiting for a chance to go.
      if (number_of_writers_pending_ > 0) {
        // Wake a waiting writer.
        // Not a problem if somebody else already did this before the waiting
        // writer got a chance to take it because it'll still return success.
        rwlock_assert_success(semGive(write_ready_));
      }
    }
  }
}

void RWLock::AddLock() {
  assert(!intContext());
  // TODO: Replace this with just atomically incrementing the right number once
  // we start using a GCC new enough to have the nice atomic builtins.
  // That will be safe because whether we're currently reading or writing can't
  // change in the middle of this.
  TaskSchedulerLocker scheduler_locker;

  // We have to be reading or writing right now, but not both.
  rwlock_assert((number_of_write_locks_ > 0) != (number_of_readers_ > 0));

  if (number_of_write_locks_ > 0) {  // we're currently writing
    ++number_of_write_locks_;
  } else {  // we're currently reading
    ++number_of_readers_;
  }
}

RWLock::RWLock() {
  number_of_write_locks_ = 0;
  number_of_writers_pending_ = 0;
  number_of_readers_ = 0;

  read_ready_ = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
  rwlock_assert(read_ready_ != NULL);
  write_ready_ = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
  rwlock_assert(write_ready_ != NULL);
}

RWLock::~RWLock() {
  // Make sure that nobody else currently has a lock or will ever be able to.
  Lock(true);

  rwlock_assert_success(semDelete(read_ready_));
  rwlock_assert_success(semDelete(write_ready_));
}
