#include "aos/common/logging/logging_impl.h"

#include <functional>

#include "aos/queue_primitives.h"

namespace aos {
namespace logging {

template <class T>
void DoLogMatrix(log_level level, const ::std::string &message,
                 const T &matrix) {
  LogImplementation::DoLogMatrix(level, message, TypeID<typename T::Scalar>::id,
                                 matrix.rows(), matrix.cols(), matrix.data());
}

}  // namespace logging
}  // namespace aos
