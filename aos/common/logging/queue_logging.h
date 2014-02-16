#ifndef AOS_COMMON_LOGGING_QUEUE_LOGGING_H_
#define AOS_COMMON_LOGGING_QUEUE_LOGGING_H_

#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "aos/common/logging/logging.h"

namespace aos {
namespace logging {

#if 1
#define LOG_STRUCT(level, message, structure)                          \
  do {                                                                 \
    static const ::std::string kAosLoggingMessage(                     \
        LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " message);         \
    ::aos::logging::DoLogStruct(level, kAosLoggingMessage, structure); \
    /* so that GCC knows that it won't return */                       \
    if (level == FATAL) {                                              \
      fprintf(stderr, "DoLogStruct(FATAL) fell through!!!!!\n");       \
      printf("see stderr\n");                                          \
      abort();                                                         \
    }                                                                  \
  } while (false)
#else
#define LOG_STRUCT(level, message, structure)
#endif

template <class T>
void DoLogStruct(log_level level, const ::std::string &message,
                 const T &structure);

}  // namespace logging
}  // namespace aos

#include "aos/common/logging/queue_logging-tmpl.h"

#endif  // AOS_COMMON_LOGGING_QUEUE_LOGGING_H_
