#ifdef __VXWORKS__
#include <taskLib.h>
#else
#include <sched.h>
#endif

#include "aos/type_traits/type_traits.h"

// It doesn't use pthread_once, because Brian looked at the pthreads
// implementation for vxworks and noticed that it is completely and entirely
// broken for doing just about anything (including its pthread_once). It has the
// same implementation under linux for simplicity.

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
  __atomic_store_n(&run_, false, __ATOMIC_SEQ_CST);
  __atomic_store_n(&done_, false, __ATOMIC_SEQ_CST);
}

template<typename T>
T *Once<T>::Get() {
  if (__atomic_exchange_n(&run_, true, __ATOMIC_RELAXED) == false) {
    result_ = function_();
    __atomic_store_n(&done_, true, __ATOMIC_RELEASE);
  } else {
    while (!__atomic_load_n(&done_, __ATOMIC_ACQUIRE)) {
      sched_yield();
    }
  }
  return result_;
}

}  // namespace aos
