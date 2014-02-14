#include "aos/common/logging/logging_impl.h"

#include <assert.h>
#include <stdarg.h>
#include <string.h>

#include "aos/common/die.h"

// This file only contains the code necessary to link (ie no implementations).
// See logging_impl.h for why this is necessary.

namespace aos {
namespace logging {
namespace internal {

LogImplementation *global_top_implementation(NULL);

Context::Context()
    : implementation(global_top_implementation),
      sequence(0) {
  cork_data.Reset();
}

void ExecuteFormat(char *output, size_t output_size,
                   const char *format, va_list ap) {
  static const char *continued = "...\n";
  const size_t size = output_size - strlen(continued);
  const int ret = vsnprintf(output, size, format, ap);
  if (ret < 0) {
    LOG(FATAL, "vsnprintf(%p, %zd, %s, args) failed with %d (%s)\n",
        output, size, format, errno, strerror(errno));
  } else if (static_cast<uintmax_t>(ret) >= static_cast<uintmax_t>(size)) {
    // Overwrite the '\0' at the end of the existing data and
    // copy in the one on the end of continued.
    memcpy(&output[size - 1], continued, strlen(continued) + 1);
  }
}

void RunWithCurrentImplementation(
    int levels, ::std::function<void(LogImplementation *)> function) {
  Context *context = Context::Get();

  LogImplementation *const top_implementation = context->implementation;
  LogImplementation *new_implementation = top_implementation;
  LogImplementation *implementation = NULL;
  assert(levels >= 1);
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

void LogImplementation::LogStruct(
    log_level level, const ::std::string &message, size_t size,
    const MessageType *type, const ::std::function<size_t(char *)> &serialize) {
  (void)level;
  (void)message;
  (void)size;
  (void)type;
  (void)serialize;
}

void LogImplementation::DoVLog(log_level level, const char *format, va_list ap,
                               int levels) {
  internal::RunWithCurrentImplementation(
      levels, [&](LogImplementation * implementation) {
    implementation->DoLog(level, format, ap);

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
