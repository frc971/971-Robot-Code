#ifndef AOS_ATOM_CODE_IPC_LIB_SYNC_H_
#define AOS_ATOM_CODE_IPC_LIB_SYNC_H_

#include <stdlib.h>
#include <signal.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// TODO(brians) add client requests to make helgrind useful with this code
// <http://www.valgrind.org/docs/manual/hg-manual.html#hg-manual.client-requests>
// and <http://www.valgrind.org/docs/manual/drd-manual.html#drd-manual.clientreqs>
// list the interesting ones

// Have to align structs containing it to sizeof(int).
// Valid initial values for use with mutex_ functions are 0 (unlocked) and 1 (locked).
// Valid initial values for use with futex_ functions are 0 (unset) and 1 (set).
// No initialization is necessary for use as c with the condition_ functions.
// The value should not be changed after multiple processes have started
// accessing an instance except through the functions declared in this file.
typedef volatile uint32_t mutex __attribute__((aligned(sizeof(int))));

// All return -1 for other error (which will be in errno from futex(2)).
//
// There is no priority inversion protection.
// TODO(brians) look at using
// <http://www.kernel.org/doc/Documentation/pi-futex.txt>

// Returns 1 if interrupted by a signal.
//
// One of the highest priority processes blocked on a given mutex will be the
// one to lock it when it is unlocked.
int mutex_lock(mutex *m) __attribute__((warn_unused_result));
// Returns 2 if it timed out or 1 if interrupted by a signal.
int mutex_lock_timeout(mutex *m, const struct timespec *timeout)
  __attribute__((warn_unused_result));
// Ignores signals. Can not fail.
int mutex_grab(mutex *m);
// abort(2)s for multiple unlocking.
void mutex_unlock(mutex *m);
// Returns 0 when successful in locking the mutex and 1 if somebody else has it
// locked.
int mutex_trylock(mutex *m) __attribute__((warn_unused_result));

// The futex_ functions are similar to the mutex_ ones but different.
// They are designed for signalling when something happens (possibly to
// multiple listeners). A mutex manipulated with them can only be set or unset.
//
// They are different from the condition_ functions in that they do NOT work
// correctly as standard condition variables. While it is possible to keep
// track of the "condition" using the value part of the futex_* functions, the
// obvious implementation has basically the same race condition that condition
// variables are designed to prevent between somebody else grabbing the mutex
// and changing whether it's set or not and the futex_ function changing the
// futex's value.

// Wait for the futex to be set. Will return immediately if it's already set.
// Returns 0 if successful or it was already set, 1 if interrupted by a signal,
// or -1.
int futex_wait(mutex *m) __attribute__((warn_unused_result));
// Set the futex and wake up anybody waiting on it.
// Returns the number that were woken or -1.
//
// This will always wake up all waiters at the same time and set the value to 1.
int futex_set(mutex *m);
// Same as above except lets something other than 1 be used as the final value.
int futex_set_value(mutex *m, mutex value);
// Unsets the futex (sets the value to 0).
// Returns 0 if it was set before and 1 if it wasn't.
// Can not fail.
int futex_unset(mutex *m);

// The condition_ functions implement condition variable support. The API is
// similar to the pthreads api and works the same way. The same m argument must
// be passed in for all calls to all of the condition_ functions with a given c.

// Wait for the condition variable to be signalled. m will be unlocked
// atomically with actually starting to wait. m is guaranteed to be locked when
// this function returns.
// NOTE: The relocking of m is not atomic with stopping the actual wait and
// other process(es) may lock (+unlock) the mutex first.
void condition_wait(mutex *c, mutex *m);
// If any other processes are condition_waiting on c, wake 1 of them. Does not
// require m to be locked.
void condition_signal(mutex *c);
// Wakes all processes that are condition_waiting on c. Does not require m to be
// locked.
void condition_broadcast(mutex *c, mutex *m);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // AOS_ATOM_CODE_IPC_LIB_SYNC_H_
