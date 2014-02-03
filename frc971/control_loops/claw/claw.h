#ifndef FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
#define FRC971_CONTROL_LOOPS_CLAW_CLAW_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/top_claw_motor_plant.h"
#include "frc971/control_loops/claw/bottom_claw_motor_plant.h"

#include "frc971/control_loops/zeroed_joint.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class ClawTest_NoWindupPositive_Test;
class ClawTest_NoWindupNegative_Test;
};

class ClawMotor
    : public aos::control_loops::ControlLoop<control_loops::ClawLoop> {
 public:
  explicit ClawMotor(
      control_loops::ClawLoop *my_claw = &control_loops::claw);

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return zeroed_joint_.capped_goal(); }

  // True if the claw is zeroing.
  bool is_zeroing() const { return zeroed_joint_.is_zeroing(); }

  // True if the claw is zeroing.
  bool is_moving_off() const { return zeroed_joint_.is_moving_off(); }

  // True if the state machine is uninitialized.
  bool is_uninitialized() const { return zeroed_joint_.is_uninitialized(); }

  // True if the state machine is ready.
  bool is_ready() const { return zeroed_joint_.is_ready(); }

 protected:
  virtual void RunIteration(
      const control_loops::ClawLoop::Goal *goal,
      const control_loops::ClawLoop::Position *position,
      control_loops::ClawLoop::Output *output,
      ::aos::control_loops::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ClawTest_NoWindupPositive_Test;
  friend class testing::ClawTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  ZeroedJoint<1> zeroed_joint_;

  DISALLOW_COPY_AND_ASSIGN(ClawMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
