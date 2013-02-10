#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>

#include <algorithm>

#include "aos/aos_core.h"
#include "aos/common/die.h"

#define DECL_LEVEL(name, value) const log_level name = value;
DECL_LEVELS
#undef DECL_LEVEL

log_level log_min = 0;

static const aos_type_sig message_sig = {sizeof(log_queue_message), 1234, 1500};
static const char *name;
static size_t name_size;
static aos_queue *queue;
static log_message corked_message;
static int cork_line_min, cork_line_max;
bool log_initted = false;

static inline void cork_init() {
  corked_message.message[0] = '\0'; // make strlen of it 0
  cork_line_min = INT_MAX;
  cork_line_max = -1;
}
int log_init(const char *name_in){
  if (log_initted) {
    return 1;
  }

  const size_t name_in_len = strlen(name_in);
  const char *last_slash = static_cast<const char *>(memrchr(name_in,
                                                                   '/', name_in_len));
  if (last_slash == NULL) {
    name_size = name_in_len;
    last_slash = name_in - 1;
  } else {
    name_size = name_in + name_in_len - last_slash;
  }
  if (name_size >= sizeof(log_message::name)) {
    fprintf(stderr, "logging: error: name '%s' (going to use %zu bytes) is too long\n",
        name_in, name_size);
    return -1;
  }
  char *const tmp = static_cast<char *>(malloc(name_size + 1));
  if (tmp == NULL) {
    fprintf(stderr, "logging: error: couldn't malloc(%zd)\n", name_size + 1);
    return -1;
  }
  name = tmp;
  memcpy(tmp, last_slash + 1, name_size);
  tmp[name_size] = 0;
  queue = aos_fetch_queue("LoggingQueue", &message_sig);
  if (queue == NULL) {
    fprintf(stderr, "logging: error: couldn't fetch queue\n");
    return -1;
  }

  cork_init();

  log_initted = true;
  return 0;
}
void log_uninit() {
  free(const_cast<char *>(name));
  name = NULL;
  name_size = 0;
  queue = NULL;
  log_initted = false;
}

static inline void check_init() {
	if (!log_initted) {
		fprintf(stderr, "logging: warning: not initialized in %jd."
            " initializing using \"<null>\" as name\n", static_cast<intmax_t>(getpid()));
		log_init("<null>");
	}
}

const log_message *log_read_next2(int flags, int *index) {
  check_init();
  return static_cast<const log_message *>(aos_queue_read_msg_index(queue, flags, index));
}
const log_message *log_read_next1(int flags) {
	check_init();
	const log_message *r = NULL;
	do {
		r = static_cast<const log_message *>(aos_queue_read_msg(queue, flags));
	} while ((flags & BLOCK) && r == NULL); // not blocking means return a NULL if that's what it gets
	return r;
}
void log_free_message(const log_message *msg) {
	check_init();
	aos_queue_free_msg(queue, msg);
}

int log_crio_message_send(log_crio_message &to_send) {
	check_init();

	log_crio_message *msg = static_cast<log_crio_message *>(aos_queue_get_msg(queue));
	if (msg == NULL) {
		fprintf(stderr, "logging: error: couldn't get a message to send '%s'\n",
            to_send.message);
		return -1;
	}
  //*msg = to_send;
  static_assert(sizeof(to_send) == sizeof(*msg), "something is very wrong here");
  memcpy(msg, &to_send, sizeof(to_send));
	if (aos_queue_write_msg(queue, msg, OVERRIDE) < 0) {
		fprintf(stderr, "logging: error: writing crio message '%s' failed\n", msg->message);
		aos_queue_free_msg(queue, msg);
		return -1;
	}

	return 0;
}

// Prints format (with ap) into output and correctly deals with the message
// being too long etc.
// Returns whether it succeeded or not.
static inline bool vsprintf_in(char *output, size_t output_size,
                               const char *format, va_list ap) {
	static const char *continued = "...\n";
	const size_t size = output_size - strlen(continued);
	const int ret = vsnprintf(output, size, format, ap);
	if (ret < 0) {
		fprintf(stderr, "logging: error: vsnprintf failed with %d (%s)\n",
            errno, strerror(errno));
    return false;
	} else if (static_cast<uintmax_t>(ret) >= static_cast<uintmax_t>(size)) {
		// overwrite the NULL at the end of the existing one and
    // copy in the one on the end of continued
		memcpy(&output[size - 1], continued, strlen(continued) + 1);
	}
  return true;
}
static inline bool write_message(log_message *msg, log_level level) {
	msg->level = level;
	msg->source = getpid();
	memcpy(msg->name, name, name_size + 1);
	if (clock_gettime(CLOCK_REALTIME, &msg->time) == -1) {
		fprintf(stderr, "logging: warning: couldn't get the current time "
            "because of %d (%s)\n", errno, strerror(errno));
		msg->time.tv_sec = 0;
		msg->time.tv_nsec = 0;
	}

  static uint8_t local_sequence = -1;
  msg->sequence = ++local_sequence;

	if (aos_queue_write_msg(queue, msg, OVERRIDE) < 0) {
		fprintf(stderr, "logging: error: writing message '%s' failed\n", msg->message);
		aos_queue_free_msg(queue, msg);
    return false;
	}
  return true;
}
static inline int vlog_do(log_level level, const char *format, va_list ap) {
	log_message *msg = static_cast<log_message *>(aos_queue_get_msg(queue));
	if (msg == NULL) {
		fprintf(stderr, "logging: error: couldn't get a message to send '%s'\n", format);
		return -1;
	}

  if (!vsprintf_in(msg->message, sizeof(msg->message), format, ap)) {
    return -1;
  }

  if (!write_message(msg, level)) {
    return -1;
  }

  if (level == FATAL) {
    aos::Die("%s", msg->message);
  }

	return 0;
}
int log_do(log_level level, const char *format, ...) {
	check_init();
	va_list ap;
	va_start(ap, format);
	const int ret = vlog_do(level, format, ap);
	va_end(ap);
  return ret;
}

static inline int vlog_cork(int line, const char *format, va_list ap) {
  const size_t message_length = strlen(corked_message.message);
  if (line > cork_line_max) cork_line_max = line;
  if (line < cork_line_min) cork_line_min = line;
  return vsprintf_in(corked_message.message + message_length,
                     sizeof(corked_message.message) - message_length, format, ap) ? 0 : -1;
}
int log_cork(int line, const char *format, ...) {
  check_init();
	va_list ap;
	va_start(ap, format);
	const int ret = vlog_cork(line, format, ap);
	va_end(ap);
  return ret;
}
static inline bool log_uncork_helper(char *output, size_t output_size,
                                     const char *format, ...) {
  check_init();
	va_list ap;
	va_start(ap, format);
	const bool ret = vsprintf_in(output, output_size, format, ap);
	va_end(ap);
  return ret;
}
int log_uncork(int line, log_level level, const char *begin_format,
               const char *format, ...) {
  check_init();
	va_list ap;
	va_start(ap, format);
	const int ret = vlog_cork(line, format, ap);
	va_end(ap);
  if (ret != 0) {
    return ret;
  }

	log_message *msg = static_cast<log_message *>(aos_queue_get_msg(queue));
	if (msg == NULL) {
		fprintf(stderr, "logging: error: couldn't get a message to send '%s'\n", format);
    cork_init();
		return -1;
  }

  static char new_format[LOG_MESSAGE_LEN];
  if (!log_uncork_helper(new_format, sizeof(new_format), begin_format,
                         cork_line_min, cork_line_max)) {
    cork_init();
    return -1;
  }
  const size_t new_length = strlen(new_format);
  memcpy(msg->message, new_format, new_length);
  memcpy(msg->message + new_length, corked_message.message,
          std::min(strlen(corked_message.message) + 1,
              sizeof(msg->message) - new_length));
  // in case corked_message.message was too long, it'll still be NULL-terminated
  msg->message[sizeof(msg->message) - 1] = '\0';
  cork_init();

  if (!write_message(msg, level)) {
    return -1;
  }

  return 0;
}

