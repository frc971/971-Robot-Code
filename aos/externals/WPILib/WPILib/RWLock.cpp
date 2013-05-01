/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include <taskLib.h>
#include <intLib.h>
#include <assert.h>
#include <tickLib.h>

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

RWLock::Locker::Locker(RWLock *lock, bool write)
  : lock_(lock), num_(lock_->Lock(write)) {
}

RWLock::Locker::Locker(const Locker &other)
  : lock_(other.lock_), num_(lock_->AddLock()) {
}

RWLock::Locker::~Locker() {
  lock_->Unlock(num_);
}

// RWLock is implemented by just locking the scheduler while doing anything
// because that is the only way under vxworks to do much of anything atomically.

RWLock::RWLock()
  : number_of_write_locks_(0),
    number_of_writers_pending_(0),
    number_of_readers_(0),
    reader_tasks_(),
    read_ready_(semBCreate(SEM_Q_PRIORITY, SEM_EMPTY)),
    write_ready_(semBCreate(SEM_Q_PRIORITY, SEM_EMPTY)) {
  rwlock_assert(read_ready_ != NULL);
  rwlock_assert(write_ready_ != NULL);
}

RWLock::~RWLock() {
  // Make sure that nobody else currently has a lock or will ever be able to.
  Lock(true);

  rwlock_assert_success(semDelete(read_ready_));
  rwlock_assert_success(semDelete(write_ready_));
}

int RWLock::Lock(bool write) {
  assert(!intContext());

  int current_task = taskIdSelf();
  // It's safe to do this check up here (outside of locking the scheduler)
  // because we only care whether the current task is in there or not and that
  // can't be changed because it's the task doing the checking.
  bool current_task_holds_already = TaskOwns(current_task);

  TaskSchedulerLocker scheduler_locker;

  taskSafe();

  // We can't be reading and writing at the same time.
  rwlock_assert(!((number_of_write_locks_ > 0) && (number_of_readers_ > 0)));

  if (write) {
    assert(!current_task_holds_already);
    // If somebody else already has it locked.
    // Don't have to worry about another task getting scheduled after
    // write_ready_ gets given because nobody else (except another writer, which
    // would just block on it) will do anything while there are pending
    // writer(s).
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
    return 0;
  } else {  // read
    // While there are one or more writers active or waiting.
    // Has to be a loop in case a writer gets scheduled between the time
    // read_ready_ gets flushed and we run.
    while ((number_of_write_locks_ > 0) || (number_of_writers_pending_ > 0)) {
      // Wait for the writer(s) to finish.
      rwlock_assert_success(semTake(read_ready_, WAIT_FOREVER));
    }

    int num = number_of_readers_;
    number_of_readers_ = num + 1;
    assert(num < kMaxReaders);
    rwlock_assert(reader_tasks_[num] == 0);
    reader_tasks_[num] = current_task;
    rwlock_assert((number_of_write_locks_ == 0) && (number_of_readers_ > 0));
    return num;
  }
}

void RWLock::Unlock(int num) {
  assert(!intContext());
  TaskSchedulerLocker scheduler_locker;

  taskUnsafe();

  // We have to be reading or writing right now, but not both.
  rwlock_assert((number_of_write_locks_ > 0) != (number_of_readers_ > 0));

  if (number_of_write_locks_ > 0) {  // we're currently writing
    rwlock_assert(num == 0);
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
        // writer got a chance to take it because it'll do nothing and return
        // success.
        rwlock_assert_success(semGive(write_ready_));
      }
    }
  } else {  // we're curently reading
    rwlock_assert(reader_tasks_[num] == taskIdSelf());
    reader_tasks_[num] = 0;
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

int RWLock::AddLock() {
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
    return 0;
  } else {  // we're currently reading
    return number_of_readers_++;
  }
}

bool RWLock::TaskOwns(int task_id) {
  for (size_t i = 0;
       i < sizeof(reader_tasks_) / sizeof(reader_tasks_[0]);
       ++i) {
    if (reader_tasks_[i] == task_id) return true;
  }
  return false;
}

#include <stdint.h>

#include "Task.h"

namespace {
namespace testing {

// It's kind of hard to test for race conditions because (by definition) they
// only happen with really specific (and uncommon) timing. However, what tests
// can cover is "normal" functioning (locking/unlocking by multiple tasks).

// How long to wait until "everything" will be done doing whatever it's going
// to.
const int kSettleTicks = 10;

void SetUp() {
}

void TearDown() {
}

struct LockerConfig {
  RWLock *const lock;
  const bool write;

  const int delay_ticks;

  bool started;
  bool locked;
  bool done;
  bool unlocked;

  LockerConfig(RWLock *lock, bool write, int delay_ticks = kSettleTicks)
      : lock(lock), write(write), delay_ticks(delay_ticks),
        started(false), locked(false), done(false), unlocked(false) {}
};
void LockerTask(LockerConfig *config) {
  config->started = true;
  {
    RWLock::Locker locker(config->lock, config->write);
    config->locked = true;
    taskDelay(config->delay_ticks);
    config->done = true;
  }
  config->unlocked = true;
}

// A basic test to make sure that 2 readers can get to it at the same time.
// Mostly just to make sure that the test setup etc works.
bool TwoReaders() {
  Task one("R1", reinterpret_cast<FUNCPTR>(LockerTask));
  Task two("R2", reinterpret_cast<FUNCPTR>(LockerTask));
  RWLock lock;

  LockerConfig one_config(&lock, false), two_config(&lock, false);
  one.Start(reinterpret_cast<uintptr_t>(&one_config));
  two.Start(reinterpret_cast<uintptr_t>(&two_config));
  while (!one_config.locked) taskDelay(1);
  assert(!one_config.done);
  while (!two_config.locked) taskDelay(1);
  if (one_config.done) {
    printf("It took too long for the second one to lock.\n");
    return false;
  }
  return true;
}

// Makes sure that everything works correctly even if a task is deleted while
// a lock is held.
bool DeleteWhileLocked() {
  Task reader("reader", reinterpret_cast<FUNCPTR>(LockerTask));
  Task writer("writer", reinterpret_cast<FUNCPTR>(LockerTask));
  static const unsigned int kDelayTicks = 15;
  RWLock lock;

  LockerConfig reader_config(&lock, false, kDelayTicks);
  LockerConfig writer_config(&lock, true, kDelayTicks);

  ULONG start = tickGet();
  reader.Start(reinterpret_cast<uintptr_t>(&reader_config));
  while (!reader_config.locked) taskDelay(1);
  writer.Start(reinterpret_cast<uintptr_t>(&writer_config));
  reader.Stop();
  if (tickGet() - start < kDelayTicks) {
    printf("Reader stopped too quickly.\n");
    return false;
  }

  while (!writer_config.done) taskDelay(1);
  if (tickGet() - start < kDelayTicks * 2) {
    printf("Writer finished too quickly.\n");
    return false;
  }
  return true;
}

#define RUN_TEST(name) do { \
  SetUp(); \
  bool test_succeeded = name(); \
  TearDown(); \
  if (!test_succeeded) successful = false; \
} while (false)
extern "C" int rwlock_test() {
  bool successful = true;

  RUN_TEST(TwoReaders);
  RUN_TEST(DeleteWhileLocked);

  return successful ? 0 : -1;
}
#undef RUN_TEST

}  // namespace testing
}  // namespace
