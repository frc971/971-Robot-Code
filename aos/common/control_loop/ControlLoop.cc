#include "aos/common/control_loop/ControlLoop.h"

namespace aos {
namespace control_loops {

time::Time NextLoopTime(time::Time start) {
  return (start / kLoopFrequency.ToNSec()) *
      kLoopFrequency.ToNSec() +
      kLoopFrequency;
}

}  // namespace control_loops
}  // namespace aos
