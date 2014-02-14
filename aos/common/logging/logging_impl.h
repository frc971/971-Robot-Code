#ifndef AOS_COMMON_LOGGING_LOGGING_IMPL_H_
#define AOS_COMMON_LOGGING_LOGGING_IMPL_H_

#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>

#include <string>
#include <functional>

#include "aos/common/logging/logging.h"
#include "aos/common/type_traits.h"
#include "aos/common/mutex.h"

namespace aos {

class MessageType;

}  // namespace aos

// This file has all of the logging implementation. It can't be #included by C
// code like logging.h can.
// It is useful for the rest of the logging implementation and other C++ code
// that needs to do special things with logging.
//
// It is implemented in logging_impl.cc and logging_interface.cc. They are
// separate so that code used by logging_impl.cc can link in
// logging_interface.cc to use logging.

namespace aos {
namespace logging {

// Unless explicitly stated otherwise, format must always be a string constant,
// args are printf-style arguments for format, and ap is a va_list of args.
// The validity of format and args together will be checked at compile time
// using a gcc function attribute.

// The struct that the code uses for making logging calls.
// Packed so that it ends up the same under both linux and vxworks.
struct __attribute__((packed)) LogMessage {
#ifdef __VXWORKS__
  static_assert(sizeof(pid_t) == sizeof(int),
                "we use task IDs (aka ints) and pid_t interchangeably");
#endif
  // Actually the task ID (aka a pointer to the TCB) on the cRIO.
  pid_t source;
  static_assert(sizeof(source) == 4, "that's how they get printed");
  // Per task/thread.
  uint16_t sequence;
  log_level level;
  int32_t seconds, nseconds;
  char name[100];
  char message[LOG_MESSAGE_LEN];
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

// Takes a message and logs it. It will set everything up and then call DoLog
// for the current LogImplementation.
void VLog(log_level level, const char *format, va_list ap);
// Adds to the saved up message.
void VCork(int line, const char *format, va_list ap);
// Actually logs the saved up message.
void VUnCork(int line, log_level level, const char *file,
             const char *format, va_list ap);

// Will call VLog with the given arguments for the next logger in the chain.
void LogNext(log_level level, const char *format, ...)
  __attribute__((format(LOG_PRINTF_FORMAT_TYPE, 2, 3)));

// Will take a structure and log it.
template <class T>
void DoLogStruct(log_level, const ::std::string &, const T &);

// Represents a system that can actually take log messages and do something
// useful with them.
// All of the code (transitively too!) in the DoLog here can make
// normal LOG and LOG_DYNAMIC calls but can NOT call LOG_CORK/LOG_UNCORK. These
// calls will not result in DoLog recursing. However, implementations must be
// safe to call from multiple threads/tasks at the same time. Also, any other
// overriden methods may end up logging through a given implementation's DoLog.
class LogImplementation {
 public:
  LogImplementation() : next_(NULL) {}

  // The one that this one's implementation logs to.
  // NULL means that there is no next one.
  LogImplementation *next() { return next_; }
  // Virtual in case a subclass wants to perform checks. There will be a valid
  // logger other than this one available while this is called.
  virtual void set_next(LogImplementation *next) { next_ = next; }

 private:
  // Actually logs the given message. Implementations should somehow create a
  // LogMessage and then call internal::FillInMessage.
  virtual void DoLog(log_level level, const char *format, va_list ap) = 0;

  // Logs the contents of an auto-generated structure. The implementation here
  // just converts it to a string with PrintMessage and then calls DoLog with
  // that, however some implementations can be a lot more efficient than that.
  // size and type are the result of calling Size() and Type() on the type of
  // the message.
  // serialize will call Serialize on the message.
  virtual void LogStruct(log_level level, const ::std::string &message,
                         size_t size, const MessageType *type,
                         const ::std::function<size_t(char *)> &serialize);

  // These functions call similar methods on the "current" LogImplementation or
  // Die if they can't find one.
  // levels is how many LogImplementations to not use off the stack.
  static void DoVLog(log_level, const char *format, va_list ap, int levels);
  // This one is implemented in queue_logging.cc.
  static void DoLogStruct(log_level level, const ::std::string &message,
                          size_t size, const MessageType *type,
                          const ::std::function<size_t(char *)> &serialize,
                          int levels);

  // Friends so that they can access the static Do* functions.
  friend void VLog(log_level, const char *, va_list);
  friend void LogNext(log_level, const char *, ...);
  template <class T>
  friend void DoLogStruct(log_level, const ::std::string &, const T &);

  LogImplementation *next_;
};

// A log implementation that dumps all messages to a C stdio stream.
class StreamLogImplementation : public LogImplementation {
 public:
  StreamLogImplementation(FILE *stream);

 private:
  virtual void DoLog(log_level level, const char *format, va_list ap);

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

extern LogImplementation *global_top_implementation;

// An separate instance of this class is accessible from each task/thread.
// NOTE: It will get deleted in the child of a fork.
struct Context {
  Context();

  // Gets the Context object for this task/thread. Will create one the first
  // time it is called.
  //
  // The implementation for each platform will lazily instantiate a new instance
  // and then initialize name the first time.
  // IMPORTANT: The implementation of this can not use logging.
  static Context *Get();
  // Deletes the Context object for this task/thread so that the next Get() is
  // called it will create a new one.
  // It is valid to call this when Get() has never been called.
  static void Delete();

  // Which one to log to right now.
  // Will be NULL if there is no logging implementation to use right now.
  LogImplementation *implementation;

  // A name representing this task/(process and thread).
  // strlen(name.c_str()) must be <= sizeof(LogMessage::name).
  ::std::string name;

  // What to assign LogMessage::source to in this task/thread.
  pid_t source;

  // The sequence value to send out with the next message.
  uint16_t sequence;

  // Contains all of the information related to implementing LOG_CORK and
  // LOG_UNCORK.
  struct {
    char message[LOG_MESSAGE_LEN];
    int line_min, line_max;
    // Sets the data up to record a new series of corked logs.
    void Reset() {
      message[0] = '\0';  // make strlen of it 0
      line_min = INT_MAX;
      line_max = -1;
      function = NULL;
    }
    // The function that the calls are in.
    // REMEMBER: While the compiler/linker will probably optimize all of the
    // identical strings to point to the same data, it might not, so using == to
    // compare this with another value is a bad idea.
    const char *function;
  } cork_data;
};

// Fills in *message according to the given inputs. Used for implementing
// LogImplementation::DoLog.
void FillInMessage(log_level level, const char *format, va_list ap,
                   LogMessage *message);

// Prints message to output.
void PrintMessage(FILE *output, const LogMessage &message);

// Prints format (with ap) into output and correctly deals with the result
// being too long etc.
void ExecuteFormat(char *output, size_t output_size, const char *format,
                   va_list ap);

// Runs the given function with the current LogImplementation (handles switching
// it out while running function etc).
void RunWithCurrentImplementation(
    int levels, ::std::function<void(LogImplementation *)> function);

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_COMMON_LOGGING_LOGGING_IMPL_H_
