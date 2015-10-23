#ifndef AOS_COMMON_LOGGING_LOGGING_H_
#define AOS_COMMON_LOGGING_LOGGING_H_

// This file contains the logging client interface. It works with both C and C++
// code.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "aos/common/macros.h"
#include "aos/common/libc/aos_strerror.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t log_level;

#define DECL_LEVELS \
DECL_LEVEL(DEBUG, 0); /* stuff that gets printed out every cycle */ \
DECL_LEVEL(INFO, 1); /* things like PosEdge/NegEdge */ \
/* things that might still work if they happen occasionally */ \
DECL_LEVEL(WARNING, 2); \
/*-1 so that vxworks macro of same name will have same effect if used*/ \
DECL_LEVEL(ERROR, -1); /* errors */ \
/* serious errors. the logging code will terminate the process/task */ \
DECL_LEVEL(FATAL, 4); \
DECL_LEVEL(LOG_UNKNOWN, 5); /* unknown logging level */
#define DECL_LEVEL(name, value) static const log_level name = value;
DECL_LEVELS;
#undef DECL_LEVEL

#ifdef __cplusplus
extern "C" {
#endif

// Actually implements the basic logging call.
// Does not check that level is valid.
void log_do(log_level level, const char *format, ...)
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 2, 3)));

void log_cork(int line, const char *function, const char *format, ...)
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 4)));
// Implements the uncork logging call.
void log_uncork(int line, const char *function, log_level level,
                const char *file, const char *format, ...)
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 5, 6)));

#ifdef __cplusplus
}
#endif

// A magical static const char[] or string literal that communicates the name
// of the enclosing function.
// It's currently using __PRETTY_FUNCTION__ because both GCC and Clang support
// that and it gives nicer results in C++ than the standard __func__ (which
// would also work).
//#define LOG_CURRENT_FUNCTION __PRETTY_FUNCTION__
#define LOG_CURRENT_FUNCTION __func__

#define LOG_SOURCENAME __FILE__

// The basic logging call.
#define LOG(level, format, args...)                                        \
  do {                                                                     \
    log_do(level, LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": %s: " format, \
           LOG_CURRENT_FUNCTION, ##args);                                  \
    /* so that GCC knows that it won't return */                           \
    if (level == FATAL) {                                                  \
      fprintf(stderr, "log_do(FATAL) fell through!!!!!\n");                \
      printf("see stderr\n");                                              \
      abort();                                                             \
    }                                                                      \
  } while (0)

// Same as LOG except appends " due to %d (%s)\n" (formatted with errno and
// aos_strerror(errno)) to the message.
#define PLOG(level, format, args...) PELOG(level, errno, format, ##args)

// Like PLOG except allows specifying an error other than errno.
#define PELOG(level, error_in, format, args...)           \
  do {                                                    \
    const int error = error_in;                           \
    LOG(level, format " due to %d (%s)\n", ##args, error, \
        aos_strerror(error));                             \
  } while (0);

// Allows format to not be a string constant.
#define LOG_DYNAMIC(level, format, args...)                             \
  do {                                                                  \
    static char log_buf[LOG_MESSAGE_LEN];                               \
    int ret = snprintf(log_buf, sizeof(log_buf), format, ##args);       \
    if (ret < 0 || (uintmax_t)ret >= LOG_MESSAGE_LEN) {                 \
      LOG(ERROR, "next message was too long so not subbing in args\n"); \
      LOG(level, "%s", format);                                         \
    } else {                                                            \
      LOG(level, "%s", log_buf);                                        \
    }                                                                   \
  } while (0)

// Allows "bottling up" multiple log fragments which can then all be logged in
// one message with LOG_UNCORK.
// Calls from a given thread/task will be grouped together.
#define LOG_CORK(format, args...)                             \
  do {                                                        \
    log_cork(__LINE__, LOG_CURRENT_FUNCTION, format, ##args); \
  } while (0)
// Actually logs all of the saved up log fragments (including format and args on
// the end).
#define LOG_UNCORK(level, format, args...)                                    \
  do {                                                                        \
    log_uncork(__LINE__, LOG_CURRENT_FUNCTION, level, LOG_SOURCENAME, format, \
               ##args);                                                       \
  } while (0)

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

namespace aos {

// CHECK* macros, similar to glog
// (<http://google-glog.googlecode.com/svn/trunk/doc/glog.html>)'s, except they
// don't support streaming in extra text. Some of the implementation is borrowed
// from there too.
// They all LOG(FATAL) with a helpful message when the check fails.
// Portions copyright (c) 1999, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// CHECK dies with a fatal error if condition is not true.  It is *not*
// controlled by NDEBUG, so the check will be executed regardless of
// compilation mode.  Therefore, it is safe to do things like:
//    CHECK(fp->Write(x) == 4)
#define CHECK(condition)                          \
  if (__builtin_expect(!(condition), 0)) {        \
    LOG(FATAL, "CHECK(%s) failed\n", #condition); \
  }

// Helper functions for CHECK_OP macro.
// The (int, int) specialization works around the issue that the compiler
// will not instantiate the template version of the function on values of
// unnamed enum type.
#define DEFINE_CHECK_OP_IMPL(name, op)                                       \
  template <typename T1, typename T2>                                        \
  inline void LogImpl##name(const T1 &v1, const T2 &v2,                      \
                            const char *exprtext) {                          \
    if (!__builtin_expect(v1 op v2, 1)) {                                    \
      log_do(FATAL,                                                          \
             LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": CHECK(%s) failed\n", \
             exprtext);                                                      \
      fprintf(stderr, "log_do(FATAL) fell through!!!!!\n");                  \
      printf("see stderr\n");                                                \
      abort();                                                               \
    }                                                                        \
  }                                                                          \
  inline void LogImpl##name(int v1, int v2, const char *exprtext) {          \
    ::aos::LogImpl##name<int, int>(v1, v2, exprtext);                        \
  }

// We use the full name Check_EQ, Check_NE, etc. in case the file including
// base/logging.h provides its own #defines for the simpler names EQ, NE, etc.
// This happens if, for example, those are used as token names in a
// yacc grammar.
DEFINE_CHECK_OP_IMPL(Check_EQ, ==)  // Compilation error with CHECK_EQ(NULL, x)?
DEFINE_CHECK_OP_IMPL(Check_NE, !=)  // Use CHECK(x == NULL) instead.
DEFINE_CHECK_OP_IMPL(Check_LE, <=)
DEFINE_CHECK_OP_IMPL(Check_LT, < )
DEFINE_CHECK_OP_IMPL(Check_GE, >=)
DEFINE_CHECK_OP_IMPL(Check_GT, > )

#define CHECK_OP(name, op, val1, val2)  \
  ::aos::LogImplCheck##name(val1, val2, \
                            STRINGIFY(val1) STRINGIFY(op) STRINGIFY(val2))

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, < , val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, > , val1, val2)

// A small helper for CHECK_NOTNULL().
template <typename T>
inline T* CheckNotNull(const char *value_name, T *t) {
  if (t == NULL) {
    LOG(FATAL, "'%s' must not be NULL\n", value_name);
  }
  return t;
}

// Check that the input is non NULL.  This very useful in constructor
// initializer lists.
#define CHECK_NOTNULL(val) ::aos::CheckNotNull(STRINGIFY(val), val)

inline int CheckSyscall(const char *syscall_string, int value) {
  if (__builtin_expect(value == -1, false)) {
    PLOG(FATAL, "%s failed", syscall_string);
  }
  return value;
}

inline void CheckSyscallReturn(const char *syscall_string, int value) {
  if (__builtin_expect(value != 0, false)) {
    PELOG(FATAL, value, "%s failed", syscall_string);
  }
}

// Check that syscall does not return -1. If it does, PLOG(FATAL)s. This is
// useful for quickly checking syscalls where it's not very useful to print out
// the values of any of the arguments. Returns the result otherwise.
//
// Example: const int fd = PCHECK(open("/tmp/whatever", O_WRONLY))
#define PCHECK(syscall) ::aos::CheckSyscall(STRINGIFY(syscall), syscall)

// PELOG(FATAL)s with the result of syscall if it returns anything other than 0.
// This is useful for quickly checking things like many of the pthreads
// functions where it's not very useful to print out the values of any of the
// arguments.
//
// Example: PRCHECK(munmap(address, length))
#define PRCHECK(syscall) ::aos::CheckSyscallReturn(STRINGIFY(syscall), syscall)

}  // namespace aos

#endif  // __cplusplus

#endif
