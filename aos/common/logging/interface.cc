#include "aos/common/logging/interface.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <type_traits>
#include <functional>

#include "aos/common/die.h"
#include "aos/common/logging/context.h"

namespace aos {
namespace logging {
namespace internal {

size_t ExecuteFormat(char *output, size_t output_size, const char *format,
                     va_list ap) {
  static const char *const continued = "...\n";
  const size_t size = output_size - strlen(continued);
  const int ret = vsnprintf(output, size, format, ap);
  typedef ::std::common_type<typeof(ret), typeof(size)>::type RetType;
  if (ret < 0) {
    PLOG(FATAL, "vsnprintf(%p, %zd, %s, args) failed",
         output, size, format);
  } else if (static_cast<RetType>(ret) >= static_cast<RetType>(size)) {
    // Overwrite the '\0' at the end of the existing data and
    // copy in the one on the end of continued.
    memcpy(&output[size - 1], continued, strlen(continued) + 1);
  }
  return ::std::min<RetType>(ret, size);
}

void RunWithCurrentImplementation(
    int levels, ::std::function<void(LogImplementation *)> function) {
  Context *context = Context::Get();

  LogImplementation *const top_implementation = context->implementation;
  LogImplementation *new_implementation = top_implementation;
  LogImplementation *implementation = NULL;
  for (int i = 0; i < levels; ++i) {
    implementation = new_implementation;
    if (new_implementation == NULL) {
      Die("no logging implementation to use\n");
    }
    new_implementation = new_implementation->next();
  }
  context->implementation = new_implementation;
  function(implementation);
  context->implementation = top_implementation;
}

}  // namespace internal

using internal::Context;

void LogImplementation::DoVLog(log_level level, const char *format, va_list ap,
                               int levels) {
  internal::RunWithCurrentImplementation(
      levels, [&](LogImplementation * implementation) {
    va_list ap1;
    va_copy(ap1, ap);
    implementation->DoLog(level, format, ap1);
    va_end(ap1);

    if (level == FATAL) {
      VDie(format, ap);
    }
  });
}

void VLog(log_level level, const char *format, va_list ap) {
  LogImplementation::DoVLog(level, format, ap, 1);
}

void VCork(int line, const char *function, const char *format, va_list ap) {
  Context *context = Context::Get();

  const size_t message_length = strlen(context->cork_data.message);
  if (line > context->cork_data.line_max) context->cork_data.line_max = line;
  if (line < context->cork_data.line_min) context->cork_data.line_min = line;

  if (context->cork_data.function == NULL) {
    context->cork_data.function = function;
  } else {
    if (strcmp(context->cork_data.function, function) != 0) {
      LOG(FATAL, "started corking data in function %s but then moved to %s\n",
          context->cork_data.function, function);
    }
  }

  internal::ExecuteFormat(context->cork_data.message + message_length,
                          sizeof(context->cork_data.message) - message_length,
                          format, ap);
}

void VUnCork(int line, const char *function, log_level level, const char *file,
             const char *format, va_list ap) {
  Context *context = Context::Get();

  VCork(line, function, format, ap);

  log_do(level, "%s: %d-%d: %s: %s", file,
         context->cork_data.line_min, context->cork_data.line_max, function,
         context->cork_data.message);

  context->cork_data.Reset();
}

}  // namespace logging
}  // namespace aos

void log_do(log_level level, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  aos::logging::VLog(level, format, ap);
  va_end(ap);
}

void log_cork(int line, const char *function, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  aos::logging::VCork(line, function, format, ap);
  va_end(ap);
}

void log_uncork(int line, const char *function, log_level level,
                const char *file, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  aos::logging::VUnCork(line, function, level, file, format, ap);
  va_end(ap);
}
