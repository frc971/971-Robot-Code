#include "aos/logging/implementations.h"

#include <inttypes.h>
#include <stdarg.h>

#include <algorithm>
#include <chrono>

#include "aos/logging/printf_formats.h"
#include "aos/time/time.h"

namespace aos {
namespace logging {
namespace internal {
namespace {

namespace chrono = ::std::chrono;

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
}

void PrintMessage(FILE *output, const LogMessage &message) {
  fprintf(output, AOS_LOGGING_BASE_FORMAT "%.*s",
          AOS_LOGGING_BASE_ARGS(message.name_length, message.name,
                                static_cast<int32_t>(message.source),
                                message.sequence, message.level,
                                message.seconds, message.nseconds),
          static_cast<int>(message.message_length), message.message);
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
  internal::Context *context = internal::Context::Get();
  context->implementation = std::move(implementation);
}

std::shared_ptr<LogImplementation> GetImplementation() {
  internal::Context *context = internal::Context::Get();
  return context->implementation;
}

}  // namespace logging
}  // namespace aos
