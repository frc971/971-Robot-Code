#ifndef AOS_COMMON_LOGGING_IMPLEMENTATIONS_H_
#define AOS_COMMON_LOGGING_IMPLEMENTATIONS_H_

#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include <string>
#include <functional>
#include <atomic>

#include "aos/common/logging/logging.h"
#include "aos/common/type_traits.h"
#include "aos/common/mutex.h"
#include "aos/common/macros.h"
#include "aos/common/logging/sizes.h"
#include "aos/common/logging/interface.h"
#include "aos/common/logging/context.h"
#include "aos/common/once.h"

namespace aos {

struct MessageType;
class RawQueue;

}  // namespace aos

// This file has various concrete LogImplementations.

namespace aos {
namespace logging {

// Unless explicitly stated otherwise, format must always be a string constant,
// args are printf-style arguments for format, and ap is a va_list of args.
// The validity of format and args together will be checked at compile time
// using a function attribute.

// Contains all of the information about a given logging call.
struct LogMessage {
  enum class Type : uint8_t {
    kString, kStruct, kMatrix
  };

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
      char
          data[LOG_MESSAGE_LEN - sizeof(type) - sizeof(rows) - sizeof(cols)];
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
#define DECL_LEVEL(name, value) if (level == name) return #name;
  DECL_LEVELS;
#undef DECL_LEVEL
  return "unknown";
}
// Returns the log level represented by str or LOG_UNKNOWN.
static inline log_level str_log(const char *str) {
#define DECL_LEVEL(name, value) if (!strcmp(str, #name)) return name;
  DECL_LEVELS;
#undef DECL_LEVEL
  return LOG_UNKNOWN;
}

// A LogImplementation where LogStruct and LogMatrix just create a string with
// PrintMessage and then forward on to DoLog.
class SimpleLogImplementation : public LogImplementation {
 private:
  void LogStruct(log_level level, const ::std::string &message, size_t size,
                 const MessageType *type,
                 const ::std::function<size_t(char *)> &serialize) override;
  void LogMatrix(log_level level, const ::std::string &message,
                 uint32_t type_id, int rows, int cols,
                 const void *data) override;
};

// Implements all of the DoLog* methods in terms of a (pure virtual in this
// class) HandleMessage method that takes a pointer to the message.
class HandleMessageLogImplementation : public LogImplementation {
 private:
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  void DoLog(log_level level, const char *format, va_list ap) override;
  void LogStruct(log_level level, const ::std::string &message_string,
                 size_t size, const MessageType *type,
                 const ::std::function<size_t(char *)> &serialize) override;
  void LogMatrix(log_level level, const ::std::string &message_string,
                 uint32_t type_id, int rows, int cols,
                 const void *data) override;

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

// Adds another implementation to the stack of implementations in this
// task/thread.
// Any tasks/threads created after this call will also use this implementation.
// The cutoff is when the state in a given task/thread is created (either lazily
// when needed or by calling Load()).
// The logging system takes ownership of implementation. It will delete it if
// necessary, so it must be created with new.
void AddImplementation(LogImplementation *implementation);

// Must be called at least once per process/load before anything else is
// called. This function is safe to call multiple times from multiple
// tasks/threads.
void Init();

// Forces all of the state that is usually lazily created when first needed to
// be created when called. Cleanup() will delete it.
void Load();

// Resets all information in this task/thread to its initial state.
// NOTE: This is not the opposite of Init(). The state that this deletes is
// lazily created when needed. It is actually the opposite of Load().
void Cleanup();

// Returns a queue which deals with LogMessage-sized messages.
// The caller takes ownership.
RawQueue *GetLoggingQueue();

// Calls AddImplementation to register the standard linux logging implementation
// which sends the messages through a queue. This implementation relies on
// another process(es) to read the log messages that it puts into the queue.
// This function is usually called by aos::Init*.
void RegisterQueueImplementation();

// This is where all of the code that is only used by actual LogImplementations
// goes.
namespace internal {

// Fills in all the parts of message according to the given inputs (with type
// kStruct).
void FillInMessageStructure(log_level level,
                            const ::std::string &message_string, size_t size,
                            const MessageType *type,
                            const ::std::function<size_t(char *)> &serialize,
                            LogMessage *message);

// Fills in all the parts of the message according to the given inputs (with
// type kMatrix).
void FillInMessageMatrix(log_level level,
                         const ::std::string &message_string, uint32_t type_id,
                         int rows, int cols, const void *data,
                         LogMessage *message);

// Fills in *message according to the given inputs (with type kString).
// Used for implementing LogImplementation::DoLog.
void FillInMessage(log_level level, const char *format, va_list ap,
                   LogMessage *message)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 2, 0)));

__attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 4)))
static inline void FillInMessageVarargs(log_level level, LogMessage *message,
                                        const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  FillInMessage(level, format, ap, message);
  va_end(ap);
}

// Prints message to output.
void PrintMessage(FILE *output, const LogMessage &message);

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_COMMON_LOGGING_IMPLEMENTATIONS_H_
