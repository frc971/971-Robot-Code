#ifndef FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_
#define FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_

#include <array>
#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"
#include "frc971/control_loops/hall_effect_loop.h"

namespace frc971 {
namespace control_loops {

// Allows the control loop to add the tests to access private members.
namespace testing {
class AngleAdjustTest_RezeroWithMissingPos_Test;
class AngleAdjustTest_DisableGoesUninitialized_Test;
}

class AngleAdjustMotor
  : public aos::control_loops::ControlLoop<control_loops::AngleAdjustLoop> {
 public:
  explicit AngleAdjustMotor(
      control_loops::AngleAdjustLoop *my_angle_adjust =
                                      &control_loops::angle_adjust);
 protected:
  virtual void RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::AngleAdjustLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status *status);

 private:
  // Allows the testing code to access some of private members.
  friend class testing::AngleAdjustTest_RezeroWithMissingPos_Test;
  friend class testing::AngleAdjustTest_DisableGoesUninitialized_Test;

  // Whether or not we are testing it currently;
  // turns on/off recording data.
  static const bool testing = true;

  // The time step of the control loop.
  static const double dt;

  // Fetches and locally caches the latest set of constants.
  // Returns whether it succeeded or not.
  bool FetchConstants();

  // Clips the goal to be inside the limits and returns the clipped goal.
  // Requires the constants to have already been fetched.
  double ClipGoal(double goal) const;
  // Limits the voltage depending whether the angle adjust has been zeroed or
  // is out of range to make it safer to use. May or may not be necessary.
  double LimitVoltage(double absolute_position, double voltage) const;

  // Hall Effect class for dealing with hall effect sensors.
  HallEffectLoop<2> hall_effect_;

  // Missed position packet count.
  int error_count_;

  // Local cache of angle adjust geometry constants.
  double lower_limit_;
  double upper_limit_;
  ::std::array<double, 2> hall_effect_stop_angle_;
  double zeroing_speed_;

  // Stores information from the queue about the hall effect sensors.
  // Because we don't have arrays in the queue, we needs these to convert
  // to a more convenient format.
  ::std::array<bool, 2> hall_effect_sensors_;
  ::std::array<double, 2> calibration_values_;

  // Time for recording data.
  double time_;

  DISALLOW_COPY_AND_ASSIGN(AngleAdjustMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_
