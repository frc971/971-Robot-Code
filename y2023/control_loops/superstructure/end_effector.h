#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_position_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {

class EndEffector {
 public:
  EndEffector();
  EndEffectorState RunIteration(
      const ::aos::monotonic_clock::time_point timestamp, bool intake,
      bool spit, bool cone_beambreak, bool cube_beambreak,
      double *intake_roller_voltage);
  EndEffectorState state();
  void Reset();

 private:
  EndEffectorState state_ = EndEffectorState::IDLE;
  aos::monotonic_clock::time_point timer_ = aos::monotonic_clock::min_time;

  bool beambreak_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
