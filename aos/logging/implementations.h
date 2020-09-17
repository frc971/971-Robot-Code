#ifndef AOS_LOGGING_IMPLEMENTATIONS_H_
#define AOS_LOGGING_IMPLEMENTATIONS_H_

#include <limits.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "aos/logging/context.h"
#include "aos/logging/interface.h"
#include "aos/logging/logging.h"
#include "aos/logging/sizes.h"
#include "aos/macros.h"
#include "aos/mutex/mutex.h"
#include "aos/once.h"
#include "aos/time/time.h"
#include "aos/type_traits/type_traits.h"

// This file has various concrete LogImplementations.

namespace aos {
namespace logging {

// Unless explicitly stated otherwise, format must always be a string constant,
// args are printf-style arguments for format, and ap is a va_list of args.
// The validity of format and args together will be checked at compile time
// using a function attribute.

// Contains all of the information about a given logging call.
struct LogMessage {
  enum class Type : uint8_t { kString };

  int32_t seconds, nseconds;
  // message_length is just the length of the actual data (which member depends
  // on the type).
  size_t message_length, name_length;
  pid_t source;
  static_assert(sizeof(source) == 4, "that's how they get printed");
  // Per task/thread.
  uint16_t sequence;
  Type type;
  log_level level;
  char name[LOG_MESSAGE_NAME_LEN];
  union {
    char message[LOG_MESSAGE_LEN];
    struct {
      uint32_t type_id;
      size_t string_length;
      // The message string and then the serialized structure.
      char serialized[LOG_MESSAGE_LEN - sizeof(type) - sizeof(string_length)];
    } structure;
    struct {
      // The type ID of the element type.
      uint32_t type;
      int rows, cols;
      size_t string_length;
      // The message string and then the serialized matrix.
      char data[LOG_MESSAGE_LEN - sizeof(type) - sizeof(rows) - sizeof(cols)];
    } matrix;
  };
};
static_assert(shm_ok<LogMessage>::value, "it's going in a queue");

// Returns left > right. LOG_UNKNOWN is most important.
static inline bool log_gt_important(log_level left, log_level right) {
  if (left == ERROR) left = 3;
  if (right == ERROR) right = 3;
  return left > right;
}

// Returns a string representing level or "unknown".
static inline const char *log_str(log_level level) {
#define DECL_LEVEL(name, value) \
  if (level == name) return #name;
  DECL_LEVELS;
#undef DECL_LEVEL
  return "unknown";
}
// Returns the log level represented by str or LOG_UNKNOWN.
static inline log_level str_log(const char *str) {
#define DECL_LEVEL(name, value) \
  if (!strcmp(str, #name)) return name;
  DECL_LEVELS;
#undef DECL_LEVEL
  return LOG_UNKNOWN;
}

// Implements all of the DoLog* methods in terms of a (pure virtual in this
// class) HandleMessage method that takes a pointer to the message.
class HandleMessageLogImplementation : public LogImplementation {
 protected:
  virtual ::aos::monotonic_clock::time_point monotonic_now() const {
    return ::aos::monotonic_clock::now();
  }

 private:
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0))) void DoLog(
      log_level level, const char *format, va_list ap) override;

  virtual void HandleMessage(const LogMessage &message) = 0;
};

// A log implementation that dumps all messages to a C stdio stream.
class StreamLogImplementation : public HandleMessageLogImplementation {
 public:
  StreamLogImplementation(FILE *stream);

 private:
  void HandleMessage(const LogMessage &message) override;

  FILE *const stream_;
};

std::shared_ptr<LogImplementation> GetImplementation();

// Sets the current implementation.
void SetImplementation(std::shared_ptr<LogImplementation> implementation);

// Updates the log implementation, returning the current implementation.
std::shared_ptr<LogImplementation> SwapImplementation(
    std::shared_ptr<LogImplementation> implementation);

// Must be called at least once per process/load before anything else is
// called. This function is safe to call multiple times from multiple
// tasks/threads.
void Init();

class CallbackLogImplementation : public HandleMessageLogImplementation {
 public:
  CallbackLogImplementation(
      const ::std::function<void(const LogMessage &)> &callback)
      : callback_(callback) {}

 private:
  void HandleMessage(const LogMessage &message) override { callback_(message); }

  ::std::function<void(const LogMessage &)> callback_;
};

// Resets all information in this task/thread to its initial state.
// NOTE: This is not the opposite of Init(). The state that this deletes is
// lazily created when needed. It is actually the opposite of Load().
void Cleanup();

void RegisterCallbackImplementation(
    const ::std::function<void(const LogMessage &)> &callback);

class ScopedLogRestorer {
 public:
  ScopedLogRestorer() = default;

  ~ScopedLogRestorer() {
    if (prev_impl_) {
      SetImplementation(std::move(prev_impl_));
    }
    Cleanup();
  }

  void Swap(std::shared_ptr<LogImplementation> new_impl) {
    prev_impl_ = SwapImplementation(std::move(new_impl));
  }

 private:
  std::shared_ptr<LogImplementation> prev_impl_;
};

// This is where all of the code that is only used by actual LogImplementations
// goes.
namespace internal {

// Fills in *message according to the given inputs (with type kString).
// Used for implementing LogImplementation::DoLog.
void FillInMessage(log_level level,
                   ::aos::monotonic_clock::time_point monotonic_now,
                   const char *format, va_list ap, LogMessage *message)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)));

__attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 4, 5))) static inline void
FillInMessageVarargs(log_level level,
                     ::aos::monotonic_clock::time_point monotonic_now,
                     LogMessage *message, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  FillInMessage(level, monotonic_now, format, ap, message);
  va_end(ap);
}

// Prints message to output.
void PrintMessage(FILE *output, const LogMessage &message);

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_LOGGING_IMPLEMENTATIONS_H_
