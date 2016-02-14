#ifndef Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/control_loops/superstructure/superstructure_controls.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {
namespace testing {
class SuperstructureTest_DisabledGoalTest_Test;
}  // namespace testing

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  // This is the angle above which we will do a HIGH_ARM_ZERO, and below which
  // we will do a LOW_ARM_ZERO.
  static constexpr double kShoulderMiddleAngle = M_PI / 4.0;
  // This is the large scale movement tolerance.
  static constexpr double kLooseTolerance = 0.05;

  // This is the small scale movement tolerance.
  static constexpr double kTightTolerance = 0.01;

  // This is the angle such that the intake will clear the arm when the shooter
  // is level.
  static constexpr double kIntakeUpperClear = 1.1;
  // This is the angle such that the intake will clear the arm when the shooter
  // is at almost any position.
  static constexpr double kIntakeLowerClear = 0.5;

  // This is the angle that the shoulder will go to when doing the
  // HIGH_ARM_ZERO.
  static constexpr double kShoulderUpAngle = M_PI / 2.0;

  // This is the angle that the shoulder will go down to when landing in the
  // bellypan.
  static constexpr double kShoulderLanded = -0.02;

  // This is the angle below which we consider the wrist close enough to level
  // that we should move it to level before doing anything.
  static constexpr double kWristAlmostLevel = 0.10;

  // This is the angle that the shoulder will go down to when raising up before
  // leveling the shooter for calibration.
  static constexpr double kShoulderWristClearAngle = 0.6;

  enum State {
    // Wait for all the filters to be ready before starting the initialization
    // process.
    UNINITIALIZED = 0,

    // We now are ready to decide how to zero.  Decide what to do once we are
    // enabled.
    DISABLED_INITIALIZED = 1,

    // Lift the arm up out of the way.
    HIGH_ARM_ZERO_LIFT_ARM = 2,

    HIGH_ARM_ZERO_LEVEL_SHOOTER = 3,

    HIGH_ARM_ZERO_MOVE_INTAKE_OUT = 4,

    HIGH_ARM_ZERO_LOWER_ARM = 6,

    LOW_ARM_ZERO_LOWER_INTAKE = 7,
    LOW_ARM_ZERO_MAYBE_LEVEL_SHOOTER = 8,
    LOW_ARM_ZERO_LIFT_SHOULDER = 9,
    LOW_ARM_ZERO_LEVEL_SHOOTER = 11,
    // Run, but limit power to zeroing voltages.
    SLOW_RUNNING = 12,
    // Run with full power.
    RUNNING = 13,
    // Internal error caused the superstructure to abort.
    ESTOP = 14,
  };

  State state() const { return state_; }

  // Returns the value to move the joint to such that it will stay below
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepBelow(double reference_angle, double current_angle,
                                 double move_distance);
  // Returns the value to move the joint to such that it will stay above
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepAbove(double reference_angle, double current_angle,
                                 double move_distance);

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  friend class testing::SuperstructureTest_DisabledGoalTest_Test;
  Intake intake_;
  Arm arm_;

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  // Returns true if the profile has finished, and the joint is within the
  // specified tolerance.
  bool IsArmNear(double tolerance);
  bool IsArmNear(double shoulder_tolerance, double wrist_tolerance);
  bool IsIntakeNear(double tolerance);

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
