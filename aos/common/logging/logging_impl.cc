#include "aos/common/logging/logging_impl.h"

#include <assert.h>
#include <stdarg.h>

#include "aos/common/time.h"
#include "aos/common/inttypes.h"
#include "aos/common/once.h"
#include "aos/common/queue_types.h"

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
namespace {

void FillInMessageBase(log_level level, LogMessage *message) {
  Context *context = Context::Get();

  message->level = level;
  message->source = context->source;
  memcpy(message->name, context->name.c_str(), context->name.size());
  message->name_length = context->name.size();

  time::Time now = time::Time::Now();
  message->seconds = now.sec();
  message->nseconds = now.nsec();

  message->sequence = context->sequence++;
}

}  // namespace

void FillInMessageStructure(log_level level,
                            const ::std::string &message_string, size_t size,
                            const MessageType *type,
                            const ::std::function<size_t(char *)> &serialize,
                            LogMessage *message) {
  type_cache::AddShm(type->id);
  message->structure.type_id = type->id;

  FillInMessageBase(level, message);

  if (message_string.size() + size > sizeof(message->structure.serialized)) {
    LOG(FATAL, "serialized struct %s (size %zd) and message %s too big\n",
        type->name.c_str(), size, message_string.c_str());
  }
  message->structure.string_length = message_string.size();
  memcpy(message->structure.serialized, message_string.data(),
         message->structure.string_length);

  message->message_length = serialize(
      &message->structure.serialized[message->structure.string_length]);
  message->type = LogMessage::Type::kStruct;
}

void FillInMessage(log_level level, const char *format, va_list ap,
                   LogMessage *message) {
  FillInMessageBase(level, message);

  message->message_length =
      ExecuteFormat(message->message, sizeof(message->message), format, ap);
  message->type = LogMessage::Type::kString;
}

void PrintMessage(FILE *output, const LogMessage &message) {
#define BASE_FORMAT \
  "%.*s(%" PRId32 ")(%05" PRIu16 "): %s at %010" PRId32 ".%09" PRId32 "s: "
#define BASE_ARGS                                             \
  static_cast<int>(message.name_length), message.name,        \
      static_cast<int32_t>(message.source), message.sequence, \
      log_str(message.level), message.seconds, message.nseconds
  switch (message.type) {
    case LogMessage::Type::kString:
      fprintf(output, BASE_FORMAT "%.*s", BASE_ARGS,
              static_cast<int>(message.message_length), message.message);
      break;
    case LogMessage::Type::kStruct:
      char buffer[LOG_MESSAGE_LEN];
      size_t output_length = sizeof(buffer);
      size_t input_length = message.message_length;
      if (!PrintMessage(
              buffer, &output_length,
              message.structure.serialized + message.structure.string_length,
              &input_length, type_cache::Get(message.structure.type_id))) {
        LOG(FATAL,
            "printing message (%.*s) of type %s into %zu-byte buffer failed\n",
            static_cast<int>(message.message_length), message.message,
            type_cache::Get(message.structure.type_id).name.c_str(),
            sizeof(buffer));
      }
      if (input_length > 0) {
        LOG(WARNING, "%zu extra bytes on message of type %s\n", input_length,
            type_cache::Get(message.structure.type_id).name.c_str());
      }
      fprintf(output, BASE_FORMAT "%.*s: %.*s\n", BASE_ARGS,
              static_cast<int>(message.message_length), message.message,
              static_cast<int>(output_length), buffer);
      break;
  }
}

}  // namespace internal

void LogImplementation::LogStruct(
    log_level level, const ::std::string &message, size_t size,
    const MessageType *type, const ::std::function<size_t(char *)> &serialize) {
  char serialized[1024];
  if (size > sizeof(serialized)) {
    LOG(FATAL, "structure of type %s too big to serialize\n",
        type->name.c_str());
  }
  size_t used = serialize(serialized);
  char printed[LOG_MESSAGE_LEN];
  size_t printed_bytes = sizeof(printed);
  if (!PrintMessage(printed, &printed_bytes, serialized, &used, *type)) {
    LOG(FATAL, "PrintMessage(%p, %p(=%zd), %p, %p(=%zd), %p(name=%s)) failed\n",
        printed, &printed_bytes, printed_bytes, serialized, &used, used, type,
        type->name.c_str());
  }
  DoLogVariadic(level, "%.*s: %.*s\n", static_cast<int>(message.size()),
                message.data(), static_cast<int>(printed_bytes), printed);
}

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
