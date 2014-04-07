#include "aos/common/controls/control_loop.h"

namespace aos {
namespace control_loops {

time::Time NextLoopTime(time::Time start) {
  return (start / static_cast<int32_t>(kLoopFrequency.ToNSec())) *
      static_cast<int32_t>(kLoopFrequency.ToNSec()) +
      kLoopFrequency;
}

}  // namespace control_loops
}  // namespace aos
