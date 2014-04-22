#ifndef AOS_COMMON_DIE_H_
#define AOS_COMMON_DIE_H_

#include <stdarg.h>

#include "aos/common/macros.h"

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

// Turns on (or off) "test mode", where (V)Die doesn't write out files and
// doesn't print to stdout.
// Test mode defaults to false.
void SetDieTestMode(bool test_mode);

}  // namespace aos

#endif  // AOS_COMMON_DIE_H_
