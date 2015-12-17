#include "aos/common/logging/matrix_logging.h"

#include "aos/common/queue_types.h"
#include "aos/common/logging/sizes.h"

namespace aos {
namespace logging {
namespace internal {

void DoLogMatrix(log_level level, const ::std::string &message,
                 uint32_t type_id, int rows, int cols, const void *data,
                 int levels) {
  {
    auto fn = [&](LogImplementation *implementation) {
      implementation->LogMatrix(level, message, type_id, rows, cols, data);
    };
    RunWithCurrentImplementation(levels, ::std::ref(fn));
  }

  if (level == FATAL) {
    char serialized[1024];
    if (static_cast<size_t>(rows * cols * MessageType::Sizeof(type_id)) >
        sizeof(serialized)) {
      Die("LOG(FATAL) matrix too big to serialize");
    }
    SerializeMatrix(type_id, serialized, data, rows, cols);
    char printed[LOG_MESSAGE_LEN];
    size_t printed_bytes = sizeof(printed);
    if (!PrintMatrix(printed, &printed_bytes, serialized, type_id, rows, cols)) {
      Die("LOG(FATAL) PrintMatrix call failed");
    }
    Die("%.*s: %.*s\n", static_cast<int>(message.size()), message.data(),
        static_cast<int>(printed_bytes), printed);
  }
}

}  // namespace internal
}  // namespace logging
}  // namespace aos
