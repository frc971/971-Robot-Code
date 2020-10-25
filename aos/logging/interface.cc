#include "aos/logging/interface.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <functional>
#include <type_traits>

#include "aos/die.h"
#include "aos/logging/context.h"

namespace aos {
namespace logging {
namespace internal {

size_t ExecuteFormat(char *output, size_t output_size, const char *format,
                     va_list ap) {
  static const char *const continued = "...\n";
  const size_t size = output_size - strlen(continued);
  const int ret = vsnprintf(output, size, format, ap);
  typedef ::std::common_type<int, size_t>::type RetType;
  if (ret < 0) {
    AOS_PLOG(FATAL, "vsnprintf(%p, %zd, %s, args) failed", output, size,
             format);
  } else if (static_cast<RetType>(ret) >= static_cast<RetType>(size)) {
    // Overwrite the '\0' at the end of the existing data and
    // copy in the one on the end of continued.
    memcpy(&output[size - 1], continued, strlen(continued) + 1);
  }
  return ::std::min<RetType>(ret, size);
}

void RunWithCurrentImplementation(
    ::std::function<void(LogImplementation *)> function) {
  Context *context = Context::Get();

  const std::shared_ptr<LogImplementation> implementation =
      context->implementation;
  if (implementation == NULL) {
    Die("no logging implementation to use\n");
  }
  function(implementation.get());
}

}  // namespace internal

using internal::Context;

void LogImplementation::DoVLog(log_level level, const char *format,
                               va_list ap) {
  auto log_impl = [&](LogImplementation *implementation) {
    va_list ap1;
    va_copy(ap1, ap);
    implementation->DoLog(level, format, ap1);
    va_end(ap1);

    if (level == FATAL) {
      VDie(format, ap);
    }
  };
  internal::RunWithCurrentImplementation(::std::ref(log_impl));
}

void VLog(log_level level, const char *format, va_list ap) {
  LogImplementation::DoVLog(level, format, ap);
}

}  // namespace logging
}  // namespace aos

void log_do(log_level level, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  aos::logging::VLog(level, format, ap);
  va_end(ap);
}
