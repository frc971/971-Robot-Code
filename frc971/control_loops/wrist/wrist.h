#ifndef FRC971_CONTROL_LOOPS_WRIST_WRIST_H_
#define FRC971_CONTROL_LOOPS_WRIST_WRIST_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/wrist/wrist_motor_plant.h"
#include "frc971/control_loops/wrist/unaugmented_wrist_motor_plant.h"

#include "frc971/control_loops/zeroed_joint.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class WristTest_NoWindupPositive_Test;
class WristTest_NoWindupNegative_Test;
};

class WristMotor
    : public aos::control_loops::ControlLoop<control_loops::WristLoop> {
 public:
  explicit WristMotor(
      control_loops::WristLoop *my_wrist = &control_loops::wrist);

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

 protected:
  virtual void RunIteration(
      const ::aos::control_loops::Goal *goal,
      const control_loops::WristLoop::Position *position,
      ::aos::control_loops::Output *output,
      ::aos::control_loops::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::WristTest_NoWindupPositive_Test;
  friend class testing::WristTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  ZeroedJoint<1> zeroed_joint_;

  DISALLOW_COPY_AND_ASSIGN(WristMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_WRIST_H_
