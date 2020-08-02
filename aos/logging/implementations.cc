#include "aos/logging/implementations.h"

#include <inttypes.h>
#include <stdarg.h>

#include <algorithm>
#include <chrono>

#include "absl/base/call_once.h"
#include "aos/die.h"
#include "aos/logging/printf_formats.h"
#include "aos/time/time.h"

namespace aos {
namespace logging {
namespace {

namespace chrono = ::std::chrono;

// The root LogImplementation. It only logs to stderr/stdout.
// Some of the things specified in the LogImplementation documentation doesn't
// apply here (mostly the parts about being able to use AOS_LOG) because this is
// the root one.
class RootLogImplementation : public LogImplementation {
 protected:
  virtual ::aos::monotonic_clock::time_point monotonic_now() const {
    return ::aos::monotonic_clock::now();
  }

 private:
  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0))) void DoLog(
      log_level level, const char *format, va_list ap) override {
    LogMessage message;
    internal::FillInMessage(level, monotonic_now(), format, ap, &message);
    internal::PrintMessage(stderr, message);
  }
};

RootLogImplementation *root_implementation = nullptr;

void SetGlobalImplementation(LogImplementation *implementation) {
  if (root_implementation == nullptr) {
    fputs("Somebody didn't call logging::Init()!\n", stderr);
    abort();
  }

  internal::Context *context = internal::Context::Get();

  context->implementation = implementation;
  internal::global_top_implementation.store(implementation);
}

void NewContext() { internal::Context::Delete(); }

void DoInit() {
  SetGlobalImplementation(root_implementation = new RootLogImplementation());

  if (pthread_atfork(NULL /*prepare*/, NULL /*parent*/, NewContext /*child*/) !=
      0) {
    AOS_LOG(FATAL, "pthread_atfork(NULL, NULL, %p) failed\n", NewContext);
  }
}

}  // namespace
namespace internal {
namespace {

void FillInMessageBase(log_level level,
                       monotonic_clock::time_point monotonic_now,
                       LogMessage *message) {
  Context *context = Context::Get();

  message->level = level;
  message->source = context->source;
  memcpy(message->name, context->name, context->name_size);
  message->name_length = context->name_size;

  message->seconds =
      chrono::duration_cast<chrono::seconds>(monotonic_now.time_since_epoch())
          .count();
  message->nseconds =
      chrono::duration_cast<chrono::nanoseconds>(
          monotonic_now.time_since_epoch() - chrono::seconds(message->seconds))
          .count();

  message->sequence = context->sequence++;
}

}  // namespace

void FillInMessage(log_level level, monotonic_clock::time_point monotonic_now,
                   const char *format, va_list ap, LogMessage *message) {
  FillInMessageBase(level, monotonic_now, message);

  message->message_length =
      ExecuteFormat(message->message, sizeof(message->message), format, ap);
  message->type = LogMessage::Type::kString;
}

void PrintMessage(FILE *output, const LogMessage &message) {
#define BASE_ARGS                                                              \
  AOS_LOGGING_BASE_ARGS(                                                       \
      message.name_length, message.name, static_cast<int32_t>(message.source), \
      message.sequence, message.level, message.seconds, message.nseconds)
  switch (message.type) {
    case LogMessage::Type::kString:
      fprintf(output, AOS_LOGGING_BASE_FORMAT "%.*s", BASE_ARGS,
              static_cast<int>(message.message_length), message.message);
      break;
  }
#undef BASE_ARGS
}

}  // namespace internal

void HandleMessageLogImplementation::DoLog(log_level level, const char *format,
                                           va_list ap) {
  LogMessage message;
  internal::FillInMessage(level, monotonic_now(), format, ap, &message);
  HandleMessage(message);
}

StreamLogImplementation::StreamLogImplementation(FILE *stream)
    : stream_(stream) {}

void StreamLogImplementation::HandleMessage(const LogMessage &message) {
  internal::PrintMessage(stream_, message);
}

void SetImplementation(LogImplementation *implementation, bool update_global) {
  internal::Context *context = internal::Context::Get();

  context->implementation = implementation;
  if (update_global) {
    SetGlobalImplementation(implementation);
  }
}

LogImplementation *SwapImplementation(LogImplementation *implementation) {
  internal::Context *context = internal::Context::Get();

  LogImplementation *old = context->implementation;
  context->implementation = implementation;

  return old;
}

LogImplementation *GetImplementation() {
  return internal::Context::Get()->implementation;
}

void Init() {
  static absl::once_flag once;
  absl::call_once(once, DoInit);
}

void Load() { internal::Context::Get(); }

void Cleanup() { internal::Context::Delete(); }

namespace {

class CallbackLogImplementation : public HandleMessageLogImplementation {
 public:
  CallbackLogImplementation(
      const ::std::function<void(const LogMessage &)> &callback)
      : callback_(callback) {}

 private:
  void HandleMessage(const LogMessage &message) override { callback_(message); }

  ::std::function<void(const LogMessage &)> callback_;
};

}  // namespace

void RegisterCallbackImplementation(
    const ::std::function<void(const LogMessage &)> &callback,
    bool update_global) {
  Init();
  SetImplementation(new CallbackLogImplementation(callback), update_global);
}

}  // namespace logging
}  // namespace aos
