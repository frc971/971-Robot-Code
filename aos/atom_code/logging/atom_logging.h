#ifndef AOS_ATOM_CODE_LOGGING_LOGGING_H_
#define AOS_ATOM_CODE_LOGGING_LOGGING_H_

// IWYU pragma: private, include "aos/common/logging/logging.h"

#ifndef AOS_COMMON_LOGGING_LOGGING_H_
#error This file may only be #included through common/logging/logging.h!!!
#endif

#include "aos/aos_core.h"

#ifdef __cplusplus
extern "C" {
#endif

int log_init(const char *name);
// WARNING: THIS LEAKS MEMORY AND SHARED MEMORY
void log_uninit(void);

extern log_level log_min;

// The basic structure that goes into the shared memory queue.
// It is packed so the pid_t at the front is at the same location as
// the one in log_crio_message.
typedef struct log_message_t_ {
	pid_t source;
	log_level level;
	char message[LOG_MESSAGE_LEN];
	char name[40];
	struct timespec time;
  uint8_t sequence; // per process
} __attribute__((packed)) log_message;

#ifdef __cplusplus
#define LOG_BOOL bool
#else
#define LOG_BOOL uint8_t
#endif
extern LOG_BOOL log_initted;
#undef LOG_BOOL

// Unless explicitly stated otherwise, format must always be a string constant
// and args are printf-style arguments for format.
// The validitiy of format and args together will be checked at compile time
// using a gcc function attribute.

// Logs the specified thing.
#define LOG(level, format, args...) do { \
	if (level >= log_min) { \
		log_do(level, LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " format, ##args); \
	} \
} while (0)
// Allows "bottling up" multiple log fragments which can then all be logged in
// one message with LOG_UNCORK.
// format does not have to be a constant
#define LOG_CORK(format, args...) do { \
  log_cork(__LINE__, format, ##args); \
} while (0)
// Actually logs all of the saved up log fragments (including format and args on
// the end).
#define LOG_UNCORK(level, format, args...) do { \
  log_uncork(__LINE__, level, LOG_SOURCENAME ": %d-%d: ", format, ##args); \
} while (0)
// Makes a normal logging call if possible or just prints it out on stderr.
#define LOG_IFINIT(level, format, args...) do{ \
	if(log_initted) { \
		LOG(level, format, args); \
	} else { \
		fprintf(stderr, "%s-noinit: " format, log_str(level), ##args); \
	} \
}while(0)

// All functions return 0 for success and - for error.

// Actually implements the basic logging call.
// Does not check that level is valid.
// TODO(brians): Fix this so it works with clang.
int log_do(log_level level, const char *format, ...)
  __attribute__((format(gnu_printf, 2, 3)));

// TODO(brians): Fix this so it works with clang.
int log_cork(int line, const char *format, ...)
  __attribute__((format(gnu_printf, 2, 3)));
// Implements the uncork logging call.
// IMPORTANT: begin_format must have 2 %d formats as its only 2 format specifiers
// which will get passed the minimum and maximum line numbers that have been
// corked into this call.
// TODO(brians): Fix this so it works with clang.
int log_uncork(int line, log_level level, const char *begin_format,
               const char *format, ...)
  __attribute__((format(gnu_printf, 4, 5)));

const log_message *log_read_next1(int flags);
const log_message *log_read_next2(int flags, int *index);
inline const log_message *log_read_next(void) { return log_read_next1(BLOCK); }
void log_free_message(const log_message *msg);

// The structure that is actually in the shared memory queue.
union log_queue_message {
  log_message atom;
  log_crio_message crio;
};

#ifdef __cplusplus
}
#endif

#endif

