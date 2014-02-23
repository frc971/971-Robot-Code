#ifndef AOS_COMMON_LOGGING_MATRIX_LOGGING_H_
#define AOS_COMMON_LOGGING_MATRIX_LOGGING_H_

#include <string>

#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/die.h"

namespace aos {
namespace logging {

// Logs the contents of a matrix and a constant string.
// matrix must be an instance of an Eigen matrix (or something similar).
#define LOG_MATRIX(level, message, matrix)                          \
  do {                                                              \
    static const ::std::string kAosLoggingMessage(                  \
        LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " message);      \
    ::aos::logging::DoLogMatrix(level, kAosLoggingMessage, matrix); \
    /* so that GCC knows that it won't return */                    \
    if (level == FATAL) {                                           \
      ::aos::Die("DoLogStruct(FATAL) fell through!!!!!\n");         \
    }                                                               \
  } while (false)

template <class T>
void DoLogMatrix(log_level level, const ::std::string &message,
                 const T &matrix);

}  // namespace logging
}  // namespace aos

#include "aos/common/logging/matrix_logging-tmpl.h"

#endif  // AOS_COMMON_LOGGING_MATRIX_LOGGING_H_
