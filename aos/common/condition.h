#ifndef AOS_COMMON_CONDITION_H_
#define AOS_COMMON_CONDITION_H_

#include "aos/common/mutex.h"
#include "aos/atom_code/ipc_lib/aos_sync.h"

namespace aos {

// A condition variable (IPC mechanism where 1 process/task can notify all
// others that are waiting for something to happen).
// This implementation will LOG(FATAL) if anything weird happens.
//
// Multiple condition variables may be associated with the same mutex but
// exactly 1 mutex must be associated with each condition variable.
class Condition {
 public:
  // m is the mutex that will be associated with this condition variable.
  explicit Condition(Mutex *m);

  // Waits for the condition variable to be signalled, atomically unlocking m at
  // the same time. The mutex associated with this condition variable must be
  // locked when this is called and will be locked when this method returns.
  void Wait();

  // Signals at most 1 other process currently Wait()ing on this condition
  // variable. Calling this does not require the mutex associated with this
  // condition variable to be locked.
  // One of the processes with the highest priority level will be woken if there
  // are multiple ones.
  void Signal();
  // Wakes all processes that are currently Wait()ing on this condition
  // variable. Calling this does not require the mutex associated with this
  // condition variable to be locked.
  void Broadcast();

 private:
  condition_variable impl_;
  Mutex *m_;
};

}  // namespace aos

#endif  // AOS_COMMON_CONDITION_H_
