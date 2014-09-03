#ifndef AOS_COMMON_DIE_H_
#define AOS_COMMON_DIE_H_

#include <stdarg.h>

#include "aos/common/macros.h"
#include "aos/common/libc/aos_strerror.h"

namespace aos {

// Terminates the task/process and logs a message (without using the logging
// framework). Designed for use in code that can't use the logging framework
// (code that can should LOG(FATAL), which calls this).
void Die(const char *format, ...)
    __attribute__((noreturn))
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 1, 2)));
void VDie(const char *format, va_list args)
    __attribute__((noreturn))
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 1, 0)));


// The same as Die except appends " because of %d (%s)" (formatted with errno
// and aos_strerror(errno)) to the message.
#define PDie(format, args...)                               \
  do {                                                      \
    const int error = errno;                                \
    ::aos::Die(format " because of %d (%s)", ##args, error, \
               aos_strerror(error));                        \
  } while (false)

// The same as Die except appends " because of %d (%s)" (formatted with error
// and aos_strerror(error)) to the message.
// PCHECK is to PDie as PRCHECK is to PRDie
//
// Example:
// const int ret = pthread_mutex_lock(whatever);
// if (ret != 0) PRDie(ret, "pthread_mutex_lock(%p) failed", whatever);
#define PRDie(error, format, args...)                       \
  do {                                                      \
    ::aos::Die(format " because of %d (%s)", ##args, error, \
               aos_strerror(error));                        \
  } while (false)

// Turns on (or off) "test mode", where (V)Die doesn't write out files and
// doesn't print to stdout.
// Test mode defaults to false.
void SetDieTestMode(bool test_mode);

}  // namespace aos

#endif  // AOS_COMMON_DIE_H_
