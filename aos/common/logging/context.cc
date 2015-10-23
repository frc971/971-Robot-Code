#include "aos/common/logging/context.h"

#include <string.h>

namespace aos {
namespace logging {
namespace internal {

::std::atomic<LogImplementation *> global_top_implementation(NULL);

Context::Context()
    : implementation(global_top_implementation.load()),
      sequence(0) {
  cork_data.Reset();
}

}  // namespace internal
}  // namespace logging
}  // namespace aos
