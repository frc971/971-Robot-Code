#ifndef Y2023_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
#define Y2023_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023_bot3/constants.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {

class EndEffector {
 public:
  static constexpr double kRollerCubeSuckVoltage() { return -7.0; }
  static constexpr double kRollerCubeSpitVoltage() { return 3.0; }
  static constexpr double kRollerCubeSpitMidVoltage() { return 5.0; }
  static constexpr double kRollerCubeSpitHighVoltage() { return 6.37; }

  EndEffector();
  void RunIteration(const ::aos::monotonic_clock::time_point timestamp,
                    RollerGoal roller_goal, bool beambreak_status,
                    double *intake_roller_voltage, bool preloaded_with_cube);
  EndEffectorState state() const { return state_; }
  void Reset();

 private:
  EndEffectorState state_;

  // variable which records the last time at which "intake" button was pressed
  aos::monotonic_clock::time_point timer_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_END_EFFECTOR_H_
