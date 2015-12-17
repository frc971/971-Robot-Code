#ifndef AOS_COMMON_LOGGING_MATRIX_LOGGING_H_
#define AOS_COMMON_LOGGING_MATRIX_LOGGING_H_

#include <string>
#include <functional>

#include "Eigen/Dense"

#include "aos/common/logging/interface.h"
#include "aos/common/die.h"
#include "aos/common/queue_primitives.h"

namespace aos {
namespace logging {

// Logs the contents of a matrix and a constant string.
// matrix must be an instance of an Eigen matrix (or something similar).
#define LOG_MATRIX(level, message, matrix)                                   \
  do {                                                                       \
    static const ::std::string kAosLoggingMessage(                           \
        LOG_SOURCENAME ": " STRINGIFY(__LINE__) ": " message);               \
    ::aos::logging::DoLogMatrixTemplated(level, kAosLoggingMessage, matrix); \
    /* so that GCC knows that it won't return */                             \
    if (level == FATAL) {                                                    \
      ::aos::Die("DoLogStruct(FATAL) fell through!!!!!\n");                  \
    }                                                                        \
  } while (false)

template <class T>
void DoLogMatrixTemplated(log_level level, const ::std::string &message,
                          const T &matrix) {
  static_assert(!T::IsRowMajor, "we only handle column-major storage");
  internal::DoLogMatrix(level, message, TypeID<typename T::Scalar>::id,
                        matrix.rows(), matrix.cols(), matrix.data(), 1);
}

}  // namespace logging
}  // namespace aos

#endif  // AOS_COMMON_LOGGING_MATRIX_LOGGING_H_
