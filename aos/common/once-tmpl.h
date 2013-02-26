#ifdef __VXWORKS__
#include <taskLib.h>
#else
#include <sched.h>
#endif

#include "aos/common/type_traits.h"

// It doesn't use pthread_once, because Brian looked at the pthreads
// implementation for vxworks and noticed that it is completely and entirely
// broken for doing just about anything (including its pthread_once). It has the
// same implementation on the atom for simplicity.

namespace aos {

// Setting function_ multiple times would be OK because it'll get set to the
// same value each time.
template<typename T>
Once<T>::Once(Function function)
    : function_(function) {
  static_assert(shm_ok<Once<T>>::value, "Once should work in shared memory");
}

template<typename T>
void Once<T>::Reset() {
  done_ = false;
  run_ = 0;
}

template<typename T>
T *Once<T>::Get() {
  if (__sync_lock_test_and_set(&run_, 1) == 0) {
    result_ = function_();
    done_ = true;
  } else {
    while (!done_) {
#ifdef __VXWORKS__
      taskDelay(1);
#else
      sched_yield();
#endif
    }
  }
  return result_;
}

}  // namespace aos
