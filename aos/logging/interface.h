#ifndef AOS_LOGGING_INTERFACE_H_
#define AOS_LOGGING_INTERFACE_H_

#include <cstdarg>
#include <functional>
#include <string>
#include <string_view>

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

// Represents a system that can actually take log messages and do something
// useful with them.
class LogImplementation {
 public:
  LogImplementation() {}

  virtual ~LogImplementation() {}

  // Returns the identifying name to be used when logging.  This could be the
  // event loop name or the thread name.
  virtual std::string_view MyName() = 0;

  // Actually logs the given message. Implementations should somehow create a
  // LogMessage and then call internal::FillInMessage.
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0))) virtual void DoLog(
      log_level level, const char *format, va_list ap) = 0;
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 4))) void DoLogVariadic(
      log_level level, const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    DoLog(level, format, ap);
    va_end(ap);
  }
};

namespace internal {

// Prints format (with ap) into output and correctly deals with the result
// being too long etc.
size_t ExecuteFormat(char *output, size_t output_size, const char *format,
                     va_list ap)
    __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)));

}  // namespace internal
}  // namespace logging
}  // namespace aos

#endif  // AOS_LOGGING_INTERFACE_H_
