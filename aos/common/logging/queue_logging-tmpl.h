#include "aos/common/logging/logging_impl.h"

#include <functional>

namespace aos {
namespace logging {

template <class T>
void DoLogStruct(log_level level, const ::std::string &message,
                 const T &structure) {
  auto fn = [&structure](char *buffer)
                -> size_t { return structure.Serialize(buffer); };

  LogImplementation::DoLogStruct(level, message, T::Size(), T::GetType(),
                                 ::std::ref(fn), 1);
}

}  // namespace logging
}  // namespace aos
