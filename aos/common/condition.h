#ifndef AOS_COMMON_CONDITION_H_
#define AOS_COMMON_CONDITION_H_

#include "aos/linux_code/ipc_lib/aos_sync.h"

namespace aos {

class Mutex;

// A condition variable (IPC mechanism where 1 process/task can notify all
// others that are waiting for something to happen) without the race condition
// where a notification is sent after some process has checked if the thing has
// happened but before it has started listening for notifications.
//
// This implementation will LOG(FATAL) if anything weird happens.
//
// A simple example of the use of a condition variable (adapted from
// pthread_cond(3)):
//
// int x, y;
// Mutex mutex;
// Condition condition(&mutex);
//
// // Waiting until x is greater than y:
// {
//   MutexLocker locker(&mutex);
//   while (!(x > y)) condition.Wait();
//   // do whatever
// }
//
// // Modifying x and/or y:
// {
//   MutexLocker locker(&mutex);
//   // modify x and y
//   if (x > y) condition.Broadcast();
// }
//
// Notice the loop around the Wait(). This is very important because some other
// process can lock the mutex and modify the shared state (possibly undoing
// whatever the Wait()er was waiting for) in between the Broadcast()er unlocking
// the mutex and the Wait()er(s) relocking it.
//
// Multiple condition variables may be associated with the same mutex but
// exactly one mutex must be associated with each condition variable.
class Condition {
 public:
  // m is the mutex that will be associated with this condition variable. This
  // object will hold on to a reference to it but does not take ownership.
  explicit Condition(Mutex *m);

  // Waits for the condition variable to be signalled, atomically unlocking the
  // mutex associated with this condition variable at the same time. The mutex
  // associated with this condition variable must be locked when this is called
  // and will be locked when this method returns.
  // NOTE: The relocking of the mutex is not performed atomically with waking
  // up.
  // Returns true if the previous owner of the mutex died before we relocked it.
  bool Wait() __attribute__((warn_unused_result));

  // Signals approximately 1 other process currently Wait()ing on this condition
  // variable. Calling this does not require the mutex associated with this
  // condition variable to be locked.
  // One of the processes with the highest priority level will be woken.
  // If there aren't any waiting at the time, none will be woken. There is a
  // small race condition in the Linux implementation that can result in more
  // than 1 being woken.
  void Signal();
  // Wakes all processes that are currently Wait()ing on this condition
  // variable. Calling this does not require the mutex associated with this
  // condition variable to be locked.
  void Broadcast();

  // Retrieves the mutex associated with this condition variable.
  Mutex *m() { return m_; }

 private:
  aos_condition impl_;
  Mutex *m_;
};

}  // namespace aos

#endif  // AOS_COMMON_CONDITION_H_
