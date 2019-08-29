#ifndef AOS_LOGGING_INTERFACE_H_
#define AOS_LOGGING_INTERFACE_H_

#include <stdarg.h>

#include <string>
#include <functional>

#include "aos/logging/logging.h"
#include "aos/macros.h"

// This file has the non-C-compatible parts of the logging client interface.

namespace aos {

struct MessageType;

namespace logging {

// Takes a message and logs it. It will set everything up and then call DoLog
// for the current LogImplementation.
void VLog(log_level level, const char *format, va_list ap)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 2, 0)));
// Adds to the saved up message.
void VCork(int line, const char *function, const char *format, va_list ap)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)));
// Actually logs the saved up message.
void VUnCork(int line, const char *function, log_level level, const char *file,
             const char *format, va_list ap)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 5, 0)));

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

  virtual bool fill_type_cache() { return true; }

 protected:
  // Actually logs the given message. Implementations should somehow create a
  // LogMessage and then call internal::FillInMessage.
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  virtual void DoLog(log_level level, const char *format, va_list ap) = 0;
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 4)))
  void DoLogVariadic(log_level level, const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    DoLog(level, format, ap);
    va_end(ap);
  }

 private:
  // These functions call similar methods on the "current" LogImplementation or
  // Die if they can't find one.
  // levels is how many LogImplementations to not use off the stack.
  static void DoVLog(log_level, const char *format, va_list ap, int levels)
      __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 2, 0)));

  friend void VLog(log_level, const char *, va_list);

  LogImplementation *next_;
};

namespace internal {

// Prints format (with ap) into output and correctly deals with the result
// being too long etc.
size_t ExecuteFormat(char *output, size_t output_size, const char *format,
                     va_list ap)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)));

// Runs the given function with the current LogImplementation (handles switching
// it out while running function etc).
// levels is how many LogImplementations to not use off the stack.
void RunWithCurrentImplementation(
    int levels, ::std::function<void(LogImplementation *)> function);

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_LOGGING_INTERFACE_H_
