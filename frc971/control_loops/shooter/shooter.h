#ifndef FRC971_CONTROL_LOOPS_shooter_shooter_H_
#define FRC971_CONTROL_LOOPS_shooter_shooter_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/control_loops/shooter/shooter_motor_plant.h"
#include "frc971/control_loops/shooter/shooter.q.h"

#include "frc971/control_loops/zeroed_joint.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class ShooterTest_NoWindupPositive_Test;
class ShooterTest_NoWindupNegative_Test;
};

class ShooterMotor
    : public aos::control_loops::ControlLoop<control_loops::ShooterLoop> {
 public:
  explicit ShooterMotor(
      control_loops::ShooterLoop *my_shooter = &control_loops::shooter_queue_group);

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return zeroed_joint_.capped_goal(); }

  // True if the shooter is zeroing.
  bool is_zeroing() const { return zeroed_joint_.is_zeroing(); }

  // True if the shooter is zeroing.
  bool is_moving_off() const { return zeroed_joint_.is_moving_off(); }

  // True if the state machine is uninitialized.
  bool is_uninitialized() const { return zeroed_joint_.is_uninitialized(); }

  // True if the state machine is ready.
  bool is_ready() const { return zeroed_joint_.is_ready(); }

 protected:
  virtual void RunIteration(
      const ShooterLoop::Goal *goal,
      const control_loops::ShooterLoop::Position *position,
      ShooterLoop::Output *output,
      ShooterLoop::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_NoWindupPositive_Test;
  friend class testing::ShooterTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  ZeroedJoint<1> zeroed_joint_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_shooter_shooter_H_
