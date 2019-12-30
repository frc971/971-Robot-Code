#include "aos/logging/implementations.h"

#include <stdarg.h>
#include <inttypes.h>

#include <algorithm>
#include <chrono>

#include "aos/die.h"
#include "aos/logging/printf_formats.h"
#include "aos/time/time.h"
#include "aos/ipc_lib/queue.h"
#include "absl/base/call_once.h"

namespace aos {
namespace logging {
namespace {

namespace chrono = ::std::chrono;

// The root LogImplementation. It only logs to stderr/stdout.
// Some of the things specified in the LogImplementation documentation doesn't
// apply here (mostly the parts about being able to use AOS_LOG) because this is
// the root one.
class RootLogImplementation : public LogImplementation {
 public:
  void have_other_implementation() { only_implementation_ = false; }

 protected:
  virtual ::aos::monotonic_clock::time_point monotonic_now() const {
    return ::aos::monotonic_clock::now();
  }

 private:
  void set_next(LogImplementation *) override {
    AOS_LOG(FATAL, "can't have a next logger from here\n");
  }

  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  void DoLog(log_level level, const char *format, va_list ap) override {
    LogMessage message;
    internal::FillInMessage(level, monotonic_now(), format, ap, &message);
    internal::PrintMessage(stderr, message);
    if (!only_implementation_) {
      fputs("root logger got used, see stderr for message\n", stdout);
    }
  }

  bool only_implementation_ = true;
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

void NewContext() {
  internal::Context::Delete();
}

void DoInit() {
  SetGlobalImplementation(root_implementation = new RootLogImplementation());

  if (pthread_atfork(NULL /*prepare*/, NULL /*parent*/,
                     NewContext /*child*/) != 0) {
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

void AddImplementation(LogImplementation *implementation) {
  internal::Context *context = internal::Context::Get();

  if (implementation->next() != NULL) {
    AOS_LOG(FATAL,
            "%p already has a next implementation, but it's not"
            " being used yet\n",
            implementation);
  }

  LogImplementation *old = context->implementation;
  if (old != NULL) {
    implementation->set_next(old);
  }
  SetGlobalImplementation(implementation);
  root_implementation->have_other_implementation();
}

void Init() {
  static absl::once_flag once;
  absl::call_once(once, DoInit);
}

void Load() {
  internal::Context::Get();
}

void Cleanup() {
  internal::Context::Delete();
}

namespace {

RawQueue *queue = NULL;

int dropped_messages = 0;
monotonic_clock::time_point dropped_start, backoff_start;
// Wait this long after dropping a message before even trying to write any more.
constexpr chrono::milliseconds kDropBackoff = chrono::milliseconds(100);

LogMessage *GetMessageOrDie() {
  LogMessage *message = static_cast<LogMessage *>(queue->GetMessage());
  if (message == NULL) {
    AOS_LOG(FATAL, "%p->GetMessage() failed\n", queue);
  } else {
    return message;
  }
}

void Write(LogMessage *msg) {
  if (__builtin_expect(dropped_messages > 0, false)) {
    monotonic_clock::time_point message_time(
        chrono::seconds(msg->seconds) + chrono::nanoseconds(msg->nseconds));
    if (message_time - backoff_start < kDropBackoff) {
      ++dropped_messages;
      queue->FreeMessage(msg);
      return;
    }

    LogMessage *dropped_message = GetMessageOrDie();
    chrono::seconds dropped_start_sec = chrono::duration_cast<chrono::seconds>(
        dropped_start.time_since_epoch());
    chrono::nanoseconds dropped_start_nsec =
        chrono::duration_cast<chrono::nanoseconds>(
            dropped_start.time_since_epoch() - dropped_start_sec);
    internal::FillInMessageVarargs(
        ERROR, message_time, dropped_message,
        "%d logs starting at %" PRId32 ".%" PRId32 " dropped\n",
        dropped_messages, static_cast<int32_t>(dropped_start_sec.count()),
        static_cast<int32_t>(dropped_start_nsec.count()));
    if (queue->WriteMessage(dropped_message, RawQueue::kNonBlock)) {
      dropped_messages = 0;
    } else {
      // Don't even bother trying to write this message because it's not likely
      // to work and it would be confusing to have one log in the middle of a
      // string of failures get through.
      ++dropped_messages;
      backoff_start = message_time;
      queue->FreeMessage(msg);
      return;
    }
  }
  if (!queue->WriteMessage(msg, RawQueue::kNonBlock)) {
    if (dropped_messages == 0) {
      monotonic_clock::time_point message_time(
          chrono::seconds(msg->seconds) + chrono::nanoseconds(msg->nseconds));
      dropped_start = backoff_start = message_time;
    }
    ++dropped_messages;
  }
}

class LinuxQueueLogImplementation : public LogImplementation {
  virtual ::aos::monotonic_clock::time_point monotonic_now() const {
    return ::aos::monotonic_clock::now();
  }

  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  void DoLog(log_level level, const char *format, va_list ap) override {
    LogMessage *message = GetMessageOrDie();
    internal::FillInMessage(level, monotonic_now(), format, ap, message);
    Write(message);
  }
};

}  // namespace

RawQueue *GetLoggingQueue() {
  return RawQueue::Fetch("LoggingQueue", sizeof(LogMessage), 1323, 10000);
}

void RegisterQueueImplementation() {
  Init();

  queue = GetLoggingQueue();
  if (queue == NULL) {
    Die("logging: couldn't fetch queue\n");
  }

  AddImplementation(new LinuxQueueLogImplementation());
}

}  // namespace logging
}  // namespace aos
