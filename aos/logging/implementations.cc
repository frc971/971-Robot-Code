#include "aos/logging/implementations.h"

#include <inttypes.h>
#include <stdarg.h>

#include <algorithm>
#include <chrono>
#include <mutex>

#include "absl/base/call_once.h"
#include "aos/die.h"
#include "aos/logging/printf_formats.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/time/time.h"

namespace aos {
namespace logging {
namespace {

namespace chrono = ::std::chrono;

struct GlobalState {
  std::shared_ptr<LogImplementation> implementation;
  aos::stl_mutex lock;
  static GlobalState *Get() {
    static GlobalState r;
    return &r;
  }
};

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

void SetImplementation(std::shared_ptr<LogImplementation> implementation) {
  Init();
  GlobalState *const global = GlobalState::Get();
  std::unique_lock<aos::stl_mutex> locker(global->lock);
  global->implementation = std::move(implementation);
}

std::shared_ptr<LogImplementation> SwapImplementation(
    std::shared_ptr<LogImplementation> implementation) {
  std::shared_ptr<LogImplementation> result;
  {
    GlobalState *const global = GlobalState::Get();
    std::unique_lock<aos::stl_mutex> locker(global->lock);
    result = std::move(global->implementation);
    global->implementation = std::move(implementation);
  }
  Cleanup();
  return result;
}

std::shared_ptr<LogImplementation> GetImplementation() {
  GlobalState *const global = GlobalState::Get();
  std::unique_lock<aos::stl_mutex> locker(global->lock);
  CHECK(global->implementation);
  return global->implementation;
}

namespace {

struct DoInit {
  DoInit() {
    GlobalState *const global = GlobalState::Get();
    std::unique_lock<aos::stl_mutex> locker(global->lock);
    CHECK(!global->implementation);
    global->implementation = std::make_shared<StreamLogImplementation>(stdout);
  }
};

}  // namespace

void Init() { static DoInit do_init; }

void Load() { internal::Context::Get(); }

void Cleanup() { internal::Context::Delete(); }

void RegisterCallbackImplementation(
    const ::std::function<void(const LogMessage &)> &callback) {
  Init();
  SetImplementation(std::make_shared<CallbackLogImplementation>(callback));
}

}  // namespace logging
}  // namespace aos
