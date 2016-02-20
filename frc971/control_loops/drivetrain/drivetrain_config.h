#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_

#include <functional>

#include "frc971/shifter_hall_effect.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

enum class ShifterType : int32_t {
  HALL_EFFECT_SHIFTER = 0,  // Detect when inbetween gears.
  SIMPLE_SHIFTER = 1,  // Switch gears without speedmatch logic.
};

struct DrivetrainConfig {
  // Shifting method we are using.
  ShifterType shifter_type;

  // Polydrivetrain functions returning various controller loops with plants.
  ::std::function<StateFeedbackLoop<4, 2, 2>()> make_drivetrain_loop;
  ::std::function<StateFeedbackLoop<2, 2, 2>()> make_v_drivetrain_loop;
  ::std::function<StateFeedbackLoop<7, 2, 3>()> make_kf_drivetrain_loop;

  double dt;  // Control loop time step.
  double robot_radius;  // Robot radius, in meters.
  double wheel_radius;  // Wheel radius, in meters.
  double v;  // Motor velocity constant.

  // Gear ratios, from wheel to motor shaft.
  double high_gear_ratio;
  double low_gear_ratio;

  // Hall effect constants. Unused if not applicable to shifter type.
  constants::ShifterHallEffect left_drive;
  constants::ShifterHallEffect right_drive;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_CONSTANTS_H_
