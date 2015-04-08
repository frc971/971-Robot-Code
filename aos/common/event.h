#ifndef AOS_COMMON_EVENT_H_
#define AOS_COMMON_EVENT_H_

#include "aos/common/time.h"

#include "aos/linux_code/ipc_lib/aos_sync.h"

namespace aos {

// An abstraction of an event which is easy to implement for Linux and in other
// environments.
// On Linux at least, this is definitely safe for passing through C code with
// memcpy etc.
//
// An event is either "set" or "unset". Any thread can transition it between
// these two states and other threads can wait for an unset->set transition.
// This is not a particularly powerful synchronization primitive, but it can be
// much easier to use than more complicated ones in some situations. The name is
// taken from Python's implementation of the same thing.
//
// An event is equivalent to a semaphore which is either set to 0 or infinity.
// It is also equivalent to a condition variable with the monitored condition
// being "is it set or not".
//
// IMPORTANT: You can NOT use this to successfully replace a standard condition
// variable in most cases. When the condition being monitored changes separately
// from the actual state of the condition variable/event, there WILL be race
// conditions if you try to use this class.
//
// It is undefined behavior to destroy an Event while there are current
// Wait()ers.
class Event {
 public:
  // Creates an unset event.
  Event();
  // There must be no waiters when an Event is destroyed.
  ~Event() = default;

  // Waits for the event to be set. Returns immediately if it is already set.
  void Wait();

  // Waits for the event to be set or until timeout has elapsed. Returns
  // immediately if it is already set.
  // Returns true if the event was Set or false if the timeout expired.
  bool WaitTimeout(const ::aos::time::Time &timeout);

  // Wakes up all Wait()ers and sets the event (atomically).
  void Set();
  // Unsets the event so future Wait() callers will block instead of returning
  // immediately.
  // Returns true if the event was previously set.
  bool Clear();

 private:
  aos_futex impl_;
};

}  // namespace aos

#endif  // AOS_COMMON_EVENT_H_
