#ifndef Y2020_CONTROL_LOOPS_SUPERSTRUCTURE_CLIMBER_H_
#define Y2020_CONTROL_LOOPS_SUPERSTRUCTURE_CLIMBER_H_

#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

// Class to encapsulate the climber logic.
class Climber {
 public:
  void Iterate(const Goal *unsafe_goal, OutputT *output);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

#endif  // Y2020_CONTROL_LOOPS_SUPERSTRUCTURE_CLIMBER_H_
