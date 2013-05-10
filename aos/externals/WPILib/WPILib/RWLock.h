/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef WPILIB_DATA_LOCK_H_
#define WPILIB_DATA_LOCK_H_

#include <semLib.h>

#include "Base.h"

/**
 * Represents a read/write lock on using some shared data so that it will not
 * be modified by any other tasks while code is using it.
 *
 * See <http://en.wikipedia.org/wiki/Readers-writer_lock> for an overview of how
 * this can be used.
 *
 * In this implementation, if there are any writers pending, then any new
 * attempts to acquire read locks will wait until all writers are done unless a
 * read lock is already held by that same task.
 */
class RWLock {
 public:
  /**
   * Represents an actual lock on the shared data. The destructor will free it.
   *
   * Intended to be used as an automatic (or local) variable so that the
   * compiler will ensure that the destructor gets called no matter how the
   * scope is exited.
   *
   * While it is possible to use new/delete to dynamically allocate an instance,
   * the constructor and destructor still MUST still be called from the same
   * task.
   *
   * Has a copy constructor which allows "copying" the lock that is held. Does
   * not have an assignment operator because assigning a lock doesn't make much
   * sense.
   */
  class Locker {
   public:
    /**
     * @param write Whether to create a writer lock (creates a reader lock
     * otherwise).
     */
    Locker(RWLock *lock, bool write);

    /**
     * Creates another lock of the same type. They can both be released
     * (destructed) independently.
     * NOTE: This does allow creating multiple write locks that are held at the
     * same time.
     */
    Locker(const Locker &other);

    /**
     * Unlocks the lock.
     */
    ~Locker();

   private:
    RWLock *const lock_;
    const int num_;

    void operator=(const Locker &);
  };

  /**
   * The maximum number of read locks that can be held at the same time.
   */
  static const int kMaxReaders = 64;

  RWLock();
  /**
   * Waits until there are no more read or write locks held.
   */
  ~RWLock();

 private:
  // The number of write locks that are currently held.
  int number_of_write_locks_;
  // How many tasks are currently waiting to get a write lock.
  // Each count in here corresponds to a task that is blocked on write_ready_.
  int number_of_writers_pending_;

  // How many read locks are currently held.
  int number_of_readers_;

  // The task ID of the task holding each read lock.
  int reader_tasks_[kMaxReaders];

  // Always locked. Gets semFlushed when readers are allowed to take the lock
  // (after all writers are done).
  SEM_ID read_ready_;
  // Locked almost all of the time. Pending writers (who have to update
  // number_of_writers_pending_) block locking this and it gets unlocked when it
  // is time for one of them to go.
  SEM_ID write_ready_;

  // Acquires the appropriate kind of lock.
  // Returns a value that must be passed to the corresponding Unlock call.
  int Lock(bool write);
  // Unlocks 1 lock.
  // Does not need to know whether it is read or write because only one type of
  // lock can be held at a time.
  // num must be the return value from the corresponding Lock/AddLock call.
  void Unlock(int num);
  // Increments the lock count by 1.
  // There must be at least 1 lock held during the execution of this method.
  // Use the regular Unlock() to unlock a lock acquired this way.
  // This is not the same as Lock(current_type) because this is the only way to
  // acquire multiple write locks at the same time.
  // Returns a value that must be passed to the corresponding Unlock call.
  int AddLock();
  // Checks whether task_id is in reader_tasks_.
  bool TaskOwns(int task_id);

  friend class Locker;

  DISALLOW_COPY_AND_ASSIGN(RWLock);
};

#endif  // WPILIB_DATA_LOCK_H_
