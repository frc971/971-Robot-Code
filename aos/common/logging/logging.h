#ifndef AOS_COMMON_LOGGING_LOGGING_H_
// must be kept in sync with crio/logging/crio_logging.h and atom_code/logging/atom_logging.h
#define AOS_COMMON_LOGGING_LOGGING_H_

#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t log_level;
#define DECL_LEVELS \
DECL_LEVEL(DEBUG, 0); /* stuff that gets printed out every cycle */ \
DECL_LEVEL(INFO, 1); /* things like PosEdge/NegEdge */ \
/* things that might still work if they happen occasionally but should be watched */ \
DECL_LEVEL(WARNING, 2); \
/*-1 so that vxworks macro of same name will have same effect if used*/ \
DECL_LEVEL(ERROR, -1); /* errors */ \
DECL_LEVEL(FATAL, 4); /* serious errors. the logging code will terminate the process/task */ \
DECL_LEVEL(LOG_UNKNOWN, 5); /* unknown logging level */
#define DECL_LEVEL(name, value) extern const log_level name;
#undef ERROR
DECL_LEVELS
#undef DECL_LEVEL

#define STRINGIFY(x) TO_STRING(x)
#define TO_STRING(x) #x

//not static const size_t for c code
#define LOG_MESSAGE_LEN 300

// Allows format to not be a string constant.
#define LOG_DYNAMIC(level, format, args...) do{ \
	static char log_buf[LOG_MESSAGE_LEN]; \
	int ret = snprintf(log_buf, sizeof(log_buf), format, ##args); \
	if(ret < 0 || (uintmax_t)ret >= LOG_MESSAGE_LEN){ \
		LOG(WARNING, "next message was too long so not subbing in args\n"); \
		LOG(level, "%s", format); \
	}else{ \
		LOG(level, "%s", log_buf); \
	} \
}while(0)

// The struct that the crio-side code uses for making logging calls.
// Packed so it's the same on the atom and the crio.
typedef struct {
  // pid_t here at the front like in log_message
  pid_t identifier; // must ALWAYS be -1 to identify that this is a crio log message
  log_level level;
  // still has to fit in LOG_MESSAGE_LEN on the atom side
	char message[LOG_MESSAGE_LEN - 50];
  double time;
  uint8_t sequence;
} __attribute__((packed)) log_crio_message;

#ifdef __cplusplus
// Just sticks the message into the shared memory queue.
int log_crio_message_send(log_crio_message &to_send);
// Returns left > right. LOG_UNKNOWN is most important.
static inline bool log_gt_important(log_level left, log_level right) {
  log_level l = left, r = right;
  if (l == ERROR) l = 3;
  if (r == ERROR) r = 3;
  return left > right;
}
#endif

// Returns a string representing level or "unknown".
static inline const char *log_str(log_level level) {
  // c doesn't really have const variables so they don't work in case statements
	if (level == DEBUG) return "DEBUG";
	if (level == INFO) return "INFO";
	if (level == WARNING) return "WARNING";
	if (level == ERROR) return "ERROR";
	if (level == FATAL) return "FATAL";
	return "unknown";
}
// Returns the log level represented by str or LOG_UNKNOWN.
static inline log_level str_log(const char *str) {
  if (!strcmp(str, "DEBUG")) return DEBUG;
  if (!strcmp(str, "INFO")) return INFO;
  if (!strcmp(str, "WARNING")) return WARNING;
  if (!strcmp(str, "ERROR")) return ERROR;
  if (!strcmp(str, "FATAL")) return FATAL;
  return LOG_UNKNOWN;
}

#ifdef __cplusplus
}
#endif

#ifdef __unix
#include "aos/atom_code/logging/atom_logging.h"  // IWYU pragma: export
#else
#include "aos/crio/logging/crio_logging.h"  // IWYU pragma: export
#endif

#endif

