#ifndef AOS_IPC_LIB_SYNC_H_
#define AOS_IPC_LIB_SYNC_H_

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

// Have to align structs containing it to to sizeof(int).
// Valid initial values for use with mutex_ functions are 0 (unlocked) and 1 (locked).
// Valid initial values for use with condition_ functions are 0 (unset) and 1 (set).
// The value should not be changed after multiple processes have started
// accessing an instance except through the functions declared in this file.
typedef volatile uint32_t mutex __attribute__((aligned(sizeof(int))));

// All return -1 for other error (which will be in errno from futex(2)).

// Returns 1 if interrupted by a signal.
int mutex_lock(mutex *m) __attribute__((warn_unused_result));
// Returns 2 if it timed out or 1 if interrupted by a signal.
int mutex_lock_timeout(mutex *m, const struct timespec *timeout)
  __attribute__((warn_unused_result));
// Ignores signals. Can not fail.
int mutex_grab(mutex *m);
// Returns 1 for multiple unlocking and -1 if something bad happened and
// whoever's waiting didn't get woken up.
int mutex_unlock(mutex *m);
// Returns 0 when successful in locking the mutex and 1 if somebody else has it
// locked.
int mutex_trylock(mutex *m) __attribute__((warn_unused_result));

// The condition_ functions are similar to the mutex_ ones but different.
// They are designed for signalling when something happens (possibly to
// multiple listeners). A mutex manipulated with them can only be set or unset.

// Wait for the condition to be set. Will return immediately if it's already set.
// Returns 0 if successful or it was already set, 1 if interrupted by a signal, or -1.
int condition_wait(mutex *m) __attribute__((warn_unused_result));
// Will wait for the next condition_set, even if the condition is already set.
// Returns 0 if successful, 1 if interrupted by a signal, or -1.
int condition_wait_force(mutex *m) __attribute__((warn_unused_result));
// Set the condition and wake up anybody waiting on it.
// Returns the number that were woken or -1.
int condition_set(mutex *m);
// Same as above except lets something other than 1 be used as the final value.
int condition_set_value(mutex *m, mutex value);
// Unsets the condition.
// Returns 0 if it was set before and 1 if it wasn't.
int condition_unset(mutex *m);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif
