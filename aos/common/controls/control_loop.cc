#include "aos/common/controls/control_loop.h"

namespace aos {
namespace controls {

time::Time NextLoopTime(time::Time start) {
  return (start / static_cast<int32_t>(kLoopFrequency.ToNSec())) *
      static_cast<int32_t>(kLoopFrequency.ToNSec()) +
      kLoopFrequency;
}

}  // namespace controls
}  // namespace aos
