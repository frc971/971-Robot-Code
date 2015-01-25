#include "aos/common/logging/logging_impl.h"

#include <stdarg.h>

#include "aos/common/time.h"
#include <inttypes.h>
#include "aos/common/once.h"
#include "aos/common/queue_types.h"
#include "aos/common/logging/logging_printf_formats.h"

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
 public:
  void have_other_implementation() { only_implementation_ = false; }

 private:
  virtual void set_next(LogImplementation *) {
    LOG(FATAL, "can't have a next logger from here\n");
  }

  __attribute__((format(GOOD_PRINTF_FORMAT_TYPE, 3, 0)))
  virtual void DoLog(log_level level, const char *format, va_list ap) {
    LogMessage message;
    internal::FillInMessage(level, format, ap, &message);
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

  Context *context = Context::Get();

  context->implementation = implementation;
  global_top_implementation.store(implementation);
}

void NewContext() {
  Context::Delete();
}

void *DoInit() {
  SetGlobalImplementation(root_implementation = new RootLogImplementation());

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
  memcpy(message->name, context->name, context->name_size);
  message->name_length = context->name_size;

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

void FillInMessageMatrix(log_level level,
                         const ::std::string &message_string, uint32_t type_id,
                         int rows, int cols, const void *data,
                         LogMessage *message) {
  CHECK(MessageType::IsPrimitive(type_id));
  message->matrix.type = type_id;

  const auto element_size = MessageType::Sizeof(type_id);

  FillInMessageBase(level, message);

  message->message_length = rows * cols * element_size;
  if (message_string.size() + message->message_length >
      sizeof(message->matrix.data)) {
    LOG(FATAL, "%dx%d matrix of type %" PRIu32
               " (size %u) and message %s is too big\n",
        rows, cols, type_id, element_size, message_string.c_str());
  }
  message->matrix.string_length = message_string.size();
  memcpy(message->matrix.data, message_string.data(),
         message->matrix.string_length);

  message->matrix.rows = rows;
  message->matrix.cols = cols;
  SerializeMatrix(type_id, &message->matrix.data[message->matrix.string_length],
                  data, rows, cols);
  message->type = LogMessage::Type::kMatrix;
}

void FillInMessage(log_level level, const char *format, va_list ap,
                   LogMessage *message) {
  FillInMessageBase(level, message);

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
    case LogMessage::Type::kStruct: {
      char buffer[2048];
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
      fprintf(output, AOS_LOGGING_BASE_FORMAT "%.*s: %.*s\n", BASE_ARGS,
              static_cast<int>(message.structure.string_length),
              message.structure.serialized,
              static_cast<int>(sizeof(buffer) - output_length), buffer);
    } break;
    case LogMessage::Type::kMatrix: {
      char buffer[1024];
      size_t output_length = sizeof(buffer);
      if (message.message_length !=
          static_cast<size_t>(message.matrix.rows * message.matrix.cols *
                              MessageType::Sizeof(message.matrix.type))) {
        LOG(FATAL, "expected %d bytes of matrix data but have %zu\n",
            message.matrix.rows * message.matrix.cols *
                MessageType::Sizeof(message.matrix.type),
            message.message_length);
      }
      if (!PrintMatrix(buffer, &output_length,
                       message.matrix.data + message.matrix.string_length,
                       message.matrix.type, message.matrix.rows,
                       message.matrix.cols)) {
        LOG(FATAL, "printing %dx%d matrix of type %" PRIu32 " failed\n",
            message.matrix.rows, message.matrix.cols, message.matrix.type);
      }
      fprintf(output, AOS_LOGGING_BASE_FORMAT "%.*s: %.*s\n", BASE_ARGS,
              static_cast<int>(message.matrix.string_length),
              message.matrix.data,
              static_cast<int>(sizeof(buffer) - output_length), buffer);
    } break;
  }
#undef BASE_ARGS
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
  char printed[1024];
  size_t printed_bytes = sizeof(printed);
  if (!PrintMessage(printed, &printed_bytes, serialized, &used, *type)) {
    LOG(FATAL, "PrintMessage(%p, %p(=%zd), %p, %p(=%zd), %p(name=%s)) failed\n",
        printed, &printed_bytes, printed_bytes, serialized, &used, used, type,
        type->name.c_str());
  }
  DoLogVariadic(level, "%.*s: %.*s\n", static_cast<int>(message.size()),
                message.data(),
                static_cast<int>(sizeof(printed) - printed_bytes), printed);
}

void LogImplementation::LogMatrix(
    log_level level, const ::std::string &message, uint32_t type_id,
    int rows, int cols, const void *data) {
  char serialized[1024];
  if (static_cast<size_t>(rows * cols * MessageType::Sizeof(type_id)) >
      sizeof(serialized)) {
    LOG(FATAL, "matrix of size %u too big to serialize\n",
        rows * cols * MessageType::Sizeof(type_id));
  }
  SerializeMatrix(type_id, serialized, data, rows, cols);
  char printed[1024];
  size_t printed_bytes = sizeof(printed);
  if (!PrintMatrix(printed, &printed_bytes, serialized, type_id, rows, cols)) {
    LOG(FATAL, "PrintMatrix(%p, %p(=%zd), %p, %" PRIu32 ", %d, %d) failed\n",
        printed, &printed_bytes, printed_bytes, serialized, type_id, rows,
        cols);
  }
  DoLogVariadic(level, "%.*s: %.*s\n", static_cast<int>(message.size()),
                message.data(),
                static_cast<int>(sizeof(printed) - printed_bytes), printed);
}

void HandleMessageLogImplementation::DoLog(log_level level, const char *format,
                                           va_list ap) {
  LogMessage message;
  internal::FillInMessage(level, format, ap, &message);
  HandleMessage(message);
}

void HandleMessageLogImplementation::LogStruct(
    log_level level, const ::std::string &message_string, size_t size,
    const MessageType *type, const ::std::function<size_t(char *)> &serialize) {
  LogMessage message;
  internal::FillInMessageStructure(level, message_string, size, type, serialize,
                                   &message);
  HandleMessage(message);
}

void HandleMessageLogImplementation::LogMatrix(
    log_level level, const ::std::string &message_string, uint32_t type_id,
    int rows, int cols, const void *data) {
  LogMessage message;
  internal::FillInMessageMatrix(level, message_string, type_id, rows, cols,
                                data, &message);
  HandleMessage(message);
}

StreamLogImplementation::StreamLogImplementation(FILE *stream)
    : stream_(stream) {}

void StreamLogImplementation::HandleMessage(const LogMessage &message) {
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
  root_implementation->have_other_implementation();
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
