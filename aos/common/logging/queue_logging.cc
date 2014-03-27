#include "aos/common/logging/queue_logging.h"

#include "aos/common/logging/logging_impl.h"
#include "aos/common/queue_types.h"

namespace aos {
namespace logging {

void LogImplementation::DoLogStruct(
    log_level level, const ::std::string &message, size_t size,
    const MessageType *type, const ::std::function<size_t(char *)> &serialize,
    int levels) {
  internal::RunWithCurrentImplementation(
      levels, [&](LogImplementation * implementation) {
    implementation->LogStruct(level, message, size, type, serialize);
  });

  if (level == FATAL) {
    char serialized[1024];
    if (size > sizeof(serialized)) {
      Die("LOG(FATAL) structure too big to serialize");
    }
    size_t used = serialize(serialized);
    char printed[LOG_MESSAGE_LEN];
    size_t printed_bytes = sizeof(printed);
    if (!PrintMessage(printed, &printed_bytes, serialized, &used, *type)) {
      Die("LOG(FATAL) PrintMessage call failed");
    }
    Die("%.*s: %.*s\n", static_cast<int>(message.size()), message.data(),
        static_cast<int>(printed_bytes), printed);
  }
}

}  // namespace logging
}  // namespace aos
