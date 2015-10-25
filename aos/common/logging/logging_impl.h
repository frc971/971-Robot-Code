#ifndef AOS_COMMON_LOGGING_LOGGING_IMPL_H_
#define AOS_COMMON_LOGGING_LOGGING_IMPL_H_

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
#include "aos/common/logging/logging_interface.h"
#include "aos/common/logging/context.h"
#include "aos/common/once.h"

namespace aos {

struct MessageType;

}  // namespace aos

// This file has all of the logging implementation. It can't be #included by C
// code like logging.h can.
// It is useful for the rest of the logging implementation and other C++ code
// that needs to do special things with logging.
//
// It is implemented in logging_impl.cc and logging_interface.cc. They are
// separate so that code used by logging_impl.cc can link in
// logging_interface.cc to use logging without creating a circular dependency.
// However, any executables with such code still need logging_impl.cc linked in.

namespace aos {
namespace logging {

// Unless explicitly stated otherwise, format must always be a string constant,
// args are printf-style arguments for format, and ap is a va_list of args.
// The validity of format and args together will be checked at compile time
// using a gcc function attribute.

// The struct that the code uses for making logging calls.
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

// Will call VLog with the given arguments for the next logger in the chain.
void LogNext(log_level level, const char *format, ...)
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 2, 3)));

// Takes a structure and log it.
template <class T>
void DoLogStruct(log_level, const ::std::string &, const T &);
// Takes a matrix and logs it.
template <class T>
void DoLogMatrix(log_level, const ::std::string &, const T &);


// Implements all of the DoLog* methods in terms of a (pure virtual in this
// class) HandleMessage method that takes a pointer to the message.
class HandleMessageLogImplementation : public LogImplementation {
 public:
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  virtual void DoLog(log_level level, const char *format, va_list ap) override;
  virtual void LogStruct(log_level level, const ::std::string &message_string,
                         size_t size, const MessageType *type,
                         const ::std::function<size_t(char *)> &serialize)
      override;
  virtual void LogMatrix(log_level level, const ::std::string &message_string,
                         uint32_t type_id, int rows, int cols, const void *data)
      override;

  virtual void HandleMessage(const LogMessage &message) = 0;
};

// A log implementation that dumps all messages to a C stdio stream.
class StreamLogImplementation : public HandleMessageLogImplementation {
 public:
  StreamLogImplementation(FILE *stream);

 private:
  virtual void HandleMessage(const LogMessage &message) override;

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

#endif  // AOS_COMMON_LOGGING_LOGGING_IMPL_H_
