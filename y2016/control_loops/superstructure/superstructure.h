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
class SuperstructureTest_RespectsRange_Test;
class SuperstructureTest_DisabledGoalTest_Test;
class SuperstructureTest_ArmZeroingErrorTest_Test;
class SuperstructureTest_IntakeZeroingErrorTest_Test;
class SuperstructureTest_UpperHardstopStartup_Test;
class SuperstructureTest_DisabledWhileZeroingHigh_Test;
class SuperstructureTest_DisabledWhileZeroingLow_Test;
}  // namespace testing

// Helper class to prevent parts from crashing into each other. The parts in
// question here are: the frame, the arm (plus shooter), and the intake.
// Assumptions:
// - The shoulder, the wrist, and intake are horizontal when at angle 0.
// - The arm (i.e. shoulder) and shooter (i.e. wrist) are stored when they are
//   both at zero degrees.
// - The intake at angle 0 is in a position to help get a ball in the robot.
// - The intake at angle PI is in a "stowed" position. In other words, it is
//   folded over the arm and shooter when they are also in a stowed position.
// - The shooter must remain horizontal when the arm is folding into the robot.
//   Otherwise, the shooter will collide with the frame.
// - The arm has priority over the intake. If the arm wants to move in such a
//   way that interferes with the intake's movement then the intake must move
//   out of the way.
class CollisionAvoidance {
 public:
  // Set up collision avoidance for an arm and intake.
  CollisionAvoidance(Intake *intake, Arm *arm) : intake_(intake), arm_(arm) {}

  // This function accepts goals for the intake and the arm and modifies them
  // in such a way that collisions between all the different parts of the robot
  // are avoided. The modified goals are then sent to the arm and intake as
  // unprofiled goals.
  void UpdateGoal(double shoulder_angle_goal, double wrist_angle_goal,
                  double intake_angle_goal);

  // Returns true if any of the limbs and frame are somehow currently
  // interfering with one another. This is based purely on the angles that the
  // limbs are reporting.
  bool collided() const;

  // Detects collision with the specified angles. This is especially useful for
  // unit testing where we have proper ground truth for all the angles.
  static bool collided_with_given_angles(double shoulder_angle,
                                         double wrist_angle,
                                         double intake_angle);

  // The shoulder angle (in radians) below which the shooter must be in a
  // stowing position. In other words the wrist must be at angle zero if the
  // shoulder is below this angle.
  static constexpr double kMinShoulderAngleForHorizontalShooter = 0.6;

  // The shoulder angle (in radians) below which the arm and the shooter have
  // the potential to interfere with the intake.
  static constexpr double kMinShoulderAngleForIntakeUpInterference = 1.3;

  // The shoulder angle (in radians) below which the shooter should be closer to
  // level to fix the inverted case.
  // TODO(austin): Verify
  static constexpr double kMinShoulderAngleForIntakeInterference = 1.1;

  // The intake angle (in radians) above which the intake can interfere (i.e.
  // collide) with the arm and/or shooter.
  static constexpr double kMaxIntakeAngleBeforeArmInterference = 1.05;

  // The maximum absolute angle (in radians) that the wrist must be below in
  // order for the shouler to be allowed to move below
  // kMinShoulderAngleForHorizontalShooter.  In other words, only allow the arm
  // to move down into the belly pan if the shooter is horizontal, ready to
  // also be placed into the belly pan.
  static constexpr double kMaxWristAngleForSafeArmStowing = 0.05;

  // The maximum angle in radians that the wrist can be from horizontal
  // while it is near the intake.
  static constexpr double kMaxWristAngleForMovingByIntake = 0.50;
  // The minimum angle in radians that the wrist can be from horizontal
  // while it is near the intake.
  static constexpr double kMinWristAngleForMovingByIntake = -1.50;

  // The shoulder angle (in radians) below which the intake can safely move
  // into the collision zone. This is necessary when the robot wants to fold up
  // completely (i.e. stow the arm, shooter, and intake).
  static constexpr double kMaxShoulderAngleUntilSafeIntakeStowing = 0.19;

 private:
  Intake *intake_;
  Arm *arm_;
};

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  static constexpr double kZeroingVoltage = 6.0;
  static constexpr double kShooterHangingVoltage = 6.0;
  static constexpr double kShooterHangingLowVoltage = 2.0;
  static constexpr double kOperatingVoltage = 12.0;
  static constexpr double kLandingShoulderDownVoltage = -1.5;

  // This is the angle above which we will do a HIGH_ARM_ZERO, and below which
  // we will do a LOW_ARM_ZERO.
  static constexpr double kShoulderMiddleAngle = M_PI / 4.0;
  // This is the large scale movement tolerance.
  static constexpr double kLooseTolerance = 0.05;

  // This is the small scale movement tolerance.
  static constexpr double kTightTolerance = 0.03;

  // This is the angle such that the intake will clear the arm when the shooter
  // is level.
  static constexpr double kIntakeUpperClear =
      CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference;
  // This is the angle such that the intake will clear the arm when the shooter
  // is at almost any position.
  static constexpr double kIntakeLowerClear = 0.4;

  // This is the angle that the shoulder will go to when doing the
  // HIGH_ARM_ZERO.
  static constexpr double kShoulderUpAngle = M_PI / 2.0;

  // This is the angle that the shoulder will go down to when landing in the
  // bellypan.
  static constexpr double kShoulderLanded = -0.02;

  // This is the angle below which the shoulder will slowly profile down and
  // land.
  static constexpr double kShoulderTransitionToLanded = 0.10;

  // This is the angle below which we consider the wrist close enough to level
  // that we should move it to level before doing anything.
  static constexpr double kWristAlmostLevel = 0.10;

  // This is the angle that the shoulder will go down to when raising up before
  // leveling the shooter for calibration.
  static constexpr double kShoulderWristClearAngle =
      CollisionAvoidance::kMinShoulderAngleForHorizontalShooter;

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
    // Run, but limit power to zeroing voltages while landing.
    LANDING_SLOW_RUNNING = 14,
    // Run with full power while landing.
    LANDING_RUNNING = 15,
    // Internal error caused the superstructure to abort.
    ESTOP = 16,
  };

  bool IsRunning() const {
    return (state_ == SLOW_RUNNING || state_ == RUNNING ||
            state_ == LANDING_SLOW_RUNNING || state_ == LANDING_RUNNING);
  }

  State state() const { return state_; }

  // Returns the value to move the joint to such that it will stay below
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepBelow(double reference_angle, double current_angle,
                                 double move_distance);
  // Returns the value to move the joint to such that it will stay above
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepAbove(double reference_angle, double current_angle,
                                 double move_distance);

  // Returns true if anything is currently considered "collided".
  bool collided() const { return collision_avoidance_.collided(); }

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  friend class testing::SuperstructureTest_DisabledGoalTest_Test;
  friend class testing::SuperstructureTest_ArmZeroingErrorTest_Test;
  friend class testing::SuperstructureTest_IntakeZeroingErrorTest_Test;
  friend class testing::SuperstructureTest_RespectsRange_Test;
  friend class testing::SuperstructureTest_UpperHardstopStartup_Test;
  friend class testing::SuperstructureTest_DisabledWhileZeroingHigh_Test;
  friend class testing::SuperstructureTest_DisabledWhileZeroingLow_Test;
  Intake intake_;
  Arm arm_;

  CollisionAvoidance collision_avoidance_;

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  float last_shoulder_angle_ = 0.0;
  float last_wrist_angle_ = 0.0;
  float last_intake_angle_ = 0.0;

  double kill_shoulder_accumulator_ = 0.0;
  bool kill_shoulder_ = false;

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
