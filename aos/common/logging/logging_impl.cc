#include "aos/common/logging/logging_impl.h"

#include <assert.h>

#include "aos/common/die.h"
#include "aos/common/time.h"
#include "aos/common/inttypes.h"
#include "aos/common/once.h"

namespace aos {
namespace logging {
namespace {

using internal::Context;

LogImplementation *global_top_implementation(NULL);
// Just going to use a mutex instead of getting fancy because speed doesn't
// really matter when accessing global_top_implementation.
Mutex global_top_implementation_mutex;
LogImplementation *get_global_top_implementation() {
  MutexLocker locker(&global_top_implementation_mutex);
  return global_top_implementation;
}

// The root LogImplementation. It only logs to stderr/stdout.
// Some of the things specified in the LogImplementation documentation doesn't
// apply here (mostly the parts about being able to use LOG) because this is the
// root one.
class RootLogImplementation : public LogImplementation {
  virtual void set_next(LogImplementation *) {
    LOG(FATAL, "can't have a next logger from here\n");
  }

  virtual void DoLog(log_level level, const char *format, va_list ap) {
    LogMessage message;
    internal::FillInMessage(level, format, ap, &message);
    internal::PrintMessage(stderr, message);
    fputs("root logger got used, see stderr for message\n", stdout);
  }
};

void SetGlobalImplementation(LogImplementation *implementation) {
  Context *context = Context::Get();

  context->implementation = implementation;
  {
    MutexLocker locker(&global_top_implementation_mutex);
    global_top_implementation = implementation;
  }
}

// Prints format (with ap) into output and correctly deals with the result
// being too long etc.
void ExecuteFormat(char *output, size_t output_size,
                   const char *format, va_list ap) {
  static const char *continued = "...\n";
  const size_t size = output_size - strlen(continued);
  const int ret = vsnprintf(output, size, format, ap);
  if (ret < 0) {
    LOG(FATAL, "vsnprintf(%p, %zd, %s, %p) failed with %d (%s)\n",
        output, size, format, ap, errno, strerror(errno));
  } else if (static_cast<uintmax_t>(ret) >= static_cast<uintmax_t>(size)) {
    // Overwrite the '\0' at the end of the existing data and
    // copy in the one on the end of continued.
    memcpy(&output[size - 1], continued, strlen(continued) + 1);
  }
}

void NewContext() {
  Context::Delete();
}

void *DoInit() {
  SetGlobalImplementation(new RootLogImplementation());

#ifndef __VXWORKS__
  if (pthread_atfork(NULL /*prepare*/, NULL /*parent*/,
                     NewContext /*child*/) != 0) {
    LOG(FATAL, "pthread_atfork(NULL, NULL, %p) failed\n",
        NewContext);
  }
#endif

  return NULL;
}

}  // namespace
namespace internal {

Context::Context()
    : implementation(get_global_top_implementation()),
      sequence(0) {
  cork_data.Reset();
}

void FillInMessage(log_level level, const char *format, va_list ap,
                   LogMessage *message) {
  Context *context = Context::Get();

  ExecuteFormat(message->message, sizeof(message->message), format, ap);

  message->level = level;
  message->source = context->source;
  memcpy(message->name, context->name.c_str(), context->name.size() + 1);

  time::Time now = time::Time::Now();
  message->seconds = now.sec();
  message->nseconds = now.nsec();

  message->sequence = context->sequence++;
}

void PrintMessage(FILE *output, const LogMessage &message) {
  fprintf(output, "%s(%" PRId32 ")(%05" PRIu16 "): %s at"
          " %010" PRId32 ".%09" PRId32 "s: %s",
          message.name, static_cast<int32_t>(message.source), message.sequence,
          log_str(message.level), message.seconds, message.nseconds,
          message.message);
}

}  // namespace internal

void LogImplementation::DoVLog(log_level level, const char *format, va_list ap,
                   int levels) {
  Context *context = Context::Get();

  LogImplementation *top_implementation = context->implementation;
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
  implementation->DoLog(level, format, ap);
  context->implementation = top_implementation;

  if (level == FATAL) {
    VDie(format, ap);
  }
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

  ExecuteFormat(context->cork_data.message + message_length,
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

void LogNext(log_level level, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  LogImplementation::DoVLog(level, format, ap, 2);
  va_end(ap);
}

void AddImplementation(LogImplementation *implementation) {
  Context *context = Context::Get();

  if (implementation->next() != NULL) {
    LOG(FATAL, "%p already has a next implementation, but it's not"
        " being used yet\n", implementation);
  }

  LogImplementation *old = context->implementation;
  if (old != NULL) {
    implementation->set_next(old);
  }
  SetGlobalImplementation(implementation);
}

void Init() {
  static Once<void> once(DoInit);
  once.Get();
}

void Load() {
  Context::Get();
}

void Cleanup() {
  Context::Delete();
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
