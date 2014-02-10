#include "aos/common/logging/logging_impl.h"

#include <assert.h>
#include <stdarg.h>

#include "aos/common/time.h"
#include "aos/common/inttypes.h"
#include "aos/common/once.h"

namespace aos {
namespace logging {
namespace {

using internal::Context;
using internal::global_top_implementation;


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
  global_top_implementation = implementation;
}

void NewContext() {
  Context::Delete();
}

void *DoInit() {
  SetGlobalImplementation(new RootLogImplementation());

  if (pthread_atfork(NULL /*prepare*/, NULL /*parent*/,
                     NewContext /*child*/) != 0) {
    LOG(FATAL, "pthread_atfork(NULL, NULL, %p) failed\n",
        NewContext);
  }

  return NULL;
}

}  // namespace
namespace internal {

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

StreamLogImplementation::StreamLogImplementation(FILE *stream)
    : stream_(stream) {}

void StreamLogImplementation::DoLog(log_level level, const char *format,
                                    va_list ap) {
  LogMessage message;
  internal::FillInMessage(level, format, ap, &message);
  internal::PrintMessage(stream_, message);
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
