#ifndef FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_
#define FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_

#include <array>
#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"
#include "frc971/control_loops/zeroed_joint.h"

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

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return zeroed_joint_.capped_goal(); }

  // True if the wrist is zeroing.
  bool is_zeroing() const { return zeroed_joint_.is_zeroing(); }

  // True if the wrist is zeroing.
  bool is_moving_off() const { return zeroed_joint_.is_moving_off(); }

  // True if the state machine is uninitialized.
  bool is_uninitialized() const { return zeroed_joint_.is_uninitialized(); }

  // True if the state machine is ready.
  bool is_ready() const { return zeroed_joint_.is_ready(); }

 private:
  // Allows the testing code to access some of private members.
  friend class testing::AngleAdjustTest_RezeroWithMissingPos_Test;
  friend class testing::AngleAdjustTest_DisableGoesUninitialized_Test;

  // The zeroed joint to use.
  ZeroedJoint<2> zeroed_joint_;

  DISALLOW_COPY_AND_ASSIGN(AngleAdjustMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_ANGLE_ADJUST_ANGLE_ADJUST_H_
