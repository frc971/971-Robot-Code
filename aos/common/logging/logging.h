#ifndef AOS_COMMON_LOGGING_LOGGING_H_
#define AOS_COMMON_LOGGING_LOGGING_H_

// This file contains the logging client interface. It works with both C and C++
// code.

#include <stdio.h>
#include <stdint.h>

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
#undef ERROR
DECL_LEVELS;
#undef DECL_LEVEL

#define STRINGIFY(x) TO_STRING(x)
#define TO_STRING(x) #x

//not static const size_t for c code
#define LOG_MESSAGE_LEN 500

#ifdef __VXWORKS__
// We're using ancient glibc, so sticking to just what the syscall can handle is
// probably safer.
#define LOG_PRINTF_FORMAT_TYPE printf
#else
#define LOG_PRINTF_FORMAT_TYPE gnu_printf
#endif
#ifdef __cplusplus
extern "C" {
#endif
// Actually implements the basic logging call.
// Does not check that level is valid.
void log_do(log_level level, const char *format, ...)
  __attribute__((format(LOG_PRINTF_FORMAT_TYPE, 2, 3)));

void log_cork(int line, const char *function, const char *format, ...)
  __attribute__((format(LOG_PRINTF_FORMAT_TYPE, 3, 4)));
// Implements the uncork logging call.
void log_uncork(int line, const char *function, log_level level,
                const char *file, const char *format, ...)
  __attribute__((format(LOG_PRINTF_FORMAT_TYPE, 5, 6)));
#ifdef __cplusplus
}
#endif

// A magical static const char[] or string literal that communicates the name
// of the enclosing function.
// It's currently using __PRETTY_FUNCTION__ because both GCC and Clang support
// that and it gives nicer results in C++ than the standard __func__ (which
// would also work).
#define LOG_CURRENT_FUNCTION __PRETTY_FUNCTION__

// The basic logging call.
#define LOG(level, format, args...) do {\
  log_do(level, LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": %s: " format, \
         LOG_CURRENT_FUNCTION, ##args); \
  /* so that GCC knows that it won't return */ \
  if (level == FATAL) { \
    fprintf(stderr, "log_do(FATAL) fell through!!!!!\n"); \
    printf("see stderr\n"); \
    abort(); \
  } \
} while (0)

// Allows format to not be a string constant.
#define LOG_DYNAMIC(level, format, args...) do { \
	static char log_buf[LOG_MESSAGE_LEN]; \
	int ret = snprintf(log_buf, sizeof(log_buf), format, ##args); \
	if (ret < 0 || (uintmax_t)ret >= LOG_MESSAGE_LEN) { \
		LOG(ERROR, "next message was too long so not subbing in args\n"); \
		LOG(level, "%s", format); \
	}else{ \
		LOG(level, "%s", log_buf); \
	} \
} while (0)

// Allows "bottling up" multiple log fragments which can then all be logged in
// one message with LOG_UNCORK.
// Calls from a given thread/task will be grouped together.
#define LOG_CORK(format, args...) do { \
  log_cork(__LINE__, LOG_CURRENT_FUNCTION, format, ##args); \
} while (0)
// Actually logs all of the saved up log fragments (including format and args on
// the end).
#define LOG_UNCORK(level, format, args...) do { \
  log_uncork(__LINE__, LOG_CURRENT_FUNCTION, level, LOG_SOURCENAME, \
             format, ##args); \
} while (0)

// TODO(brians) add CHECK macros like glog
// (<http://google-glog.googlecode.com/svn/trunk/doc/glog.html>)
// and replace assert with one

#ifdef __cplusplus
}
#endif

#endif
