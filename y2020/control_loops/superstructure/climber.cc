#include "y2020/control_loops/superstructure/climber.h"

#include <algorithm>

#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

void Climber::Iterate(const Goal *unsafe_goal, OutputT *output) {
  if (unsafe_goal && output) {
    // Pass through the voltage request from the user.  Backwards isn't
    // supported, so prevent that.
    output->climber_voltage =
        std::clamp(unsafe_goal->climber_voltage(), 0.0f, 12.0f);
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
