#ifndef AOS_COMMON_CONDITION_H_
#define AOS_COMMON_CONDITION_H_

#ifdef __VXWORKS__
#include <semLib.h>
#endif

#include "aos/aos_core.h"
#include "aos/common/mutex.h"

namespace aos {

// A condition variable (IPC mechanism where 1 process/task can notify all
// others that are waiting for something to happen).
// There are implementations for both the atom and the cRIO.
// They both LOG(FATAL) if anything weird happens.
//
// A condition is either set or unset, and multiple processes/tasks can wait on
// one for it to be set.
class Condition {
 public:
  // Creates an unset condition.
  Condition();
#ifdef __VXWORKS__
  // Will not make sure that it is either set or unset.
  ~Condition();
#endif
  // Waits for the condition to be set. Will return true immediately if it is
  // already set.
  // Returns false if returning before a confirmed condition set, although doing
  // anything very useful with the return value is difficult because the
  // condition may have been set (and possibly even unset again) between the
  // time when the system call to block returned and this function returns.
  bool Wait();
  // Waits for the next Set(), regardless of whether or not the condition is
  // currently set.
  // Same return value as Wait().
  bool WaitNext();

  // Sets the condition. Any processes/tasks that are currently Wait()ing will
  // continue.
  // All implementations will wake all waiting processes/tasks at once so that
  // the highest priority one(s) will run before others like usual.
  void Set();
  // Unsets the condition.
  void Unset();

 private:
#ifdef __VXWORKS__
  // Always empty. Used to make tasks wait and then gets flushed to unblock all
  // of them.
  SEM_ID wait_;
  // Whether or not the conditon is set.
  bool set_;
#else
  mutex impl_;
#endif
};

}  // namespace aos

#endif  // AOS_COMMON_CONDITION_H_
