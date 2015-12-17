#ifndef AOS_COMMON_LOGGING_QUEUE_LOGGING_H_
#define AOS_COMMON_LOGGING_QUEUE_LOGGING_H_

#include <stdio.h>
#include <stdlib.h>

#include <functional>
#include <string>

#include "aos/common/logging/interface.h"
#include "aos/common/die.h"

namespace aos {
namespace logging {

// Logs the contents of a structure (or Queue message) and a constant string.
// structure must be an instance of one of the generated queue types.
#define LOG_STRUCT(level, message, structure)                       \
  do {                                                              \
    static const ::std::string kAosLoggingMessage(                  \
        LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " message);      \
    ::aos::logging::DoLogStructTemplated(level, kAosLoggingMessage, \
                                         structure);                \
    /* so that GCC knows that it won't return */                    \
    if (level == FATAL) {                                           \
      ::aos::Die("DoLogStruct(FATAL) fell through!!!!!\n");         \
    }                                                               \
  } while (false)

template <class T>
void DoLogStructTemplated(log_level level, const ::std::string &message,
                          const T &structure) {
  auto fn = [&structure](char *buffer)
                -> size_t { return structure.Serialize(buffer); };

  internal::DoLogStruct(level, message, T::Size(), T::GetType(), ::std::ref(fn),
                        1);
}

}  // namespace logging
}  // namespace aos

#endif  // AOS_COMMON_LOGGING_QUEUE_LOGGING_H_
