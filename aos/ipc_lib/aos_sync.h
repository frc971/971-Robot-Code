#ifndef AOS_IPC_LIB_SYNC_H_
#define AOS_IPC_LIB_SYNC_H_

#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// TODO(brians) add client requests to make helgrind useful with this code
// <http://www.valgrind.org/docs/manual/hg-manual.html#hg-manual.client-requests>
// and <http://www.valgrind.org/docs/manual/drd-manual.html#drd-manual.clientreqs>
// list the interesting ones

// Have to remember to align structs containing it (recursively) to sizeof(int).
// Valid initial values for use with futex_ functions are 0 (unset) and 1 (set).
// The value should not be changed after multiple processes have started
// accessing an instance except through the functions declared in this file.
typedef uint32_t aos_futex __attribute__((aligned(sizeof(int))));

// For use with the condition_ functions.
// No initialization is necessary.
typedef aos_futex aos_condition;

// For use with the mutex_ functions.
// futex must be initialized to 0.
// No initialization is necessary for next and previous.
// Under ThreadSanitizer, pthread_mutex_init must be initialized to false.
// The recommended way to initialize one of these is by memset(3)ing the whole
// thing to 0 or using C++ () initialization to avoid depending on the
// implementation.
struct aos_mutex {
  // 2 links to get O(1) adds and removes.
  // This is &next of another element.
  // next (might) have stuff |ed into it to indicate PI futexes and might also
  // have an offset (see SetRobustListOffset); previous is an actual pointer
  // without any of that.
  // next has to stay the first element of this structure.
  uintptr_t next;
  struct aos_mutex *previous;
  aos_futex futex;
#ifdef AOS_SANITIZER_thread
  // Internal pthread mutex which is kept in sync with the actual mutex so tsan
  // can understand what's happening and help catch bugs.
  pthread_mutex_t pthread_mutex;
#ifndef __cplusplus
  // TODO(brian): Remove this once the stupid C code is gone...
#define bool uint8_t
#endif
  bool pthread_mutex_init;
#ifndef __cplusplus
#undef bool
#endif
#endif
};

// The mutex_ functions are designed to be used as mutexes. A mutex can only be
// unlocked from the same task which originally locked it. Also, if a task dies
// while holding a mutex, the next person who locks it will be notified. After a
// fork(2), any mutexes held will be held ONLY in the parent process. Attempting
// to unlock them from the child will give errors.
// Priority inheritance (aka priority inversion protection) is enabled.

// All of these return 1 if the previous owner died with it held, 2 if
// interrupted by a signal, 3 if timed out, or 4 if an optional lock fails. Some
// of them (obviously) can never return some of those values.
//
// One of the highest priority processes blocked on a given mutex will be the
// one to lock it when it is unlocked.
int mutex_lock(struct aos_mutex *m) __attribute__((warn_unused_result));
// Returns 2 if it timed out or 1 if interrupted by a signal.
int mutex_lock_timeout(struct aos_mutex *m, const struct timespec *timeout)
    __attribute__((warn_unused_result));
// Ignores signals (retries until something other than getting a signal
// happens).
int mutex_grab(struct aos_mutex *m) __attribute__((warn_unused_result));
// LOG(FATAL)s for multiple unlocking.
void mutex_unlock(struct aos_mutex *m);
// Does not block waiting for the mutex.
int mutex_trylock(struct aos_mutex *m) __attribute__((warn_unused_result));
#ifdef __cplusplus
// Returns whether or not the mutex is locked by this thread.
// There aren't very many valid uses for this function; the main ones are
// checking mutexes as they are destroyed to catch problems with that early and
// stack-based recursive mutex locking.
bool mutex_islocked(const aos_mutex *m);
#endif

// The futex_ functions are similar to the mutex_ ones but different.
// They are designed for signalling when something happens (possibly to
// multiple listeners). A aos_futex manipulated with them can only be set or
// unset. Also, they can be set/unset/waited on from any task independently of
// who did something first and have no priority inversion protection.
// They return -1 for other error (which will be in errno from futex(2)).
// They have no spurious wakeups (because everybody always gets woken up).
//
// Another name for this kind of synchronization mechanism is a "notification".
// Python calls it an "event".
//
// They are different from the condition_ functions in that they do NOT work
// correctly as standard condition variables. While it is possible to keep
// track of the "condition" using the value part of the futex_* functions, the
// obvious implementation has basically the same race condition that condition
// variables are designed to prevent between somebody else grabbing the mutex
// and changing whether it's set or not and the futex_ function changing the
// futex's value. A futex is effectively a resettable condition variable with
// the condition being "has it been set"; if you have some other condition (for
// example messages are available to read on a queue), use the condition_
// functions or there will be race conditions.

// Wait for the futex to be set. Will return immediately if it's already set
// (after a syscall).
// Returns 0 if successful or it was already set, 1 if interrupted by a signal,
// or -1 with an error in errno. Can return 0 spuriously.
int futex_wait(aos_futex *m) __attribute__((warn_unused_result));
// The same as futex_wait except returns 2 if it times out.
int futex_wait_timeout(aos_futex *m, const struct timespec *timeout)
  __attribute__((warn_unused_result));

// Set the futex and wake up anybody waiting on it.
// Returns the number that were woken or -1 with an error in errno.
//
// This will always wake up all waiters at the same time and set the value to 1.
int futex_set(aos_futex *m);
// Same as above except lets something other than 1 be used as the final value.
int futex_set_value(aos_futex *m, aos_futex value);
// Unsets the futex (sets the value to 0).
// Returns 0 if it was set before and 1 if it wasn't.
// Can not fail.
int futex_unset(aos_futex *m);

// The condition_ functions implement condition variable support. The API is
// similar to the pthreads api and works the same way. The same m argument must
// be passed in for all calls to all of the condition_ functions with a given c.
// They do have the potential for spurious wakeups.

// Wait for the condition variable to be signalled. m will be unlocked
// atomically with actually starting to wait. m is guaranteed to be locked when
// this function returns.
// NOTE: The relocking of m is not atomic with stopping the actual wait and
// other process(es) may lock (+unlock) the mutex first.
// Returns 0 on success, 1 if the previous owner died or -1 if we timed out.
// Will only return -1 on timeout if end_time is not null.
int condition_wait(aos_condition *c, struct aos_mutex *m,
                   struct timespec *end_time)
    __attribute__((warn_unused_result));
// If any other processes are condition_waiting on c, wake 1 of them. Does not
// require m to be locked.
// NOTE: There is a small chance that this will wake more than just 1 waiter.
void condition_signal(aos_condition *c, struct aos_mutex *m);
// Wakes all processes that are condition_waiting on c. Does not require m to be
// locked.
void condition_broadcast(aos_condition *c, struct aos_mutex *m);

#ifdef __cplusplus
}

namespace aos {
namespace linux_code {
namespace ipc_lib {

typedef void (*FutexAccessorObserver)(void *address, bool write);

// Set functions which get called before and after all futex operations.
void SetFutexAccessorObservers(FutexAccessorObserver before,
                               FutexAccessorObserver after);

// Set the offset to use for putting addresses into the robust list.
// This is necessary to work around a kernel bug where it hangs when trying to
// deal with a futex on the robust list when its memory has been changed to
// read-only.
void SetRobustListOffset(ptrdiff_t offset);

// Returns true if there are any mutexes still locked by this task.
// This is mainly useful for verifying tests don't mess up other ones by leaving
// now-freed but still locked mutexes around.
bool HaveLockedMutexes();

}  // namespace ipc_lib
}  // namespace linux_code
}  // namespace aos

#endif  // __cplusplus

#endif  // AOS_IPC_LIB_SYNC_H_
