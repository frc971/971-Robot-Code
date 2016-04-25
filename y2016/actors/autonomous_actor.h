#ifndef Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_ACTORS_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

#include "y2016/actors/autonomous_action.q.h"
#include "y2016/actors/vision_align_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2016 {
namespace actors {
using ::frc971::ProfileParameters;

class AutonomousActor
    : public ::aos::common::actions::ActorBase<AutonomousActionQueueGroup> {
 public:
  explicit AutonomousActor(AutonomousActionQueueGroup *s);

  bool RunAction(const actors::AutonomousActionParams &params) override;

 private:
  void ResetDrivetrain();
  void InitializeEncoders();
  void WaitUntilDoneOrCanceled(::std::unique_ptr<aos::common::actions::Action>
      action);
  void StartDrive(double distance, double angle,
                  ::frc971::ProfileParameters linear,
                  ::frc971::ProfileParameters angular);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();
  void WaitForBallOrDriveDone();

  void StealAndMoveOverBy(double distance);

  // Returns true if the drive has finished.
  bool IsDriveDone();
  // Waits until the robot is pitched up above the specified angle, or the move
  // finishes.  Returns true on success, and false if it cancels.
  bool WaitForAboveAngle(double angle);
  bool WaitForBelowAngle(double angle);
  bool WaitForMaxBy(double angle);

  // Waits until the profile and distance is within distance and angle of the
  // goal.  Returns true on success, and false when canceled.
  bool WaitForDriveNear(double distance, double angle);

  const ::frc971::control_loops::drivetrain::DrivetrainConfig dt_config_;

  // Initial drivetrain positions.
  struct InitialDrivetrain {
    double left;
    double right;
  };
  InitialDrivetrain initial_drivetrain_;

  // Internal struct holding superstructure goals sent by autonomous to the
  // loop.
  struct SuperstructureGoal {
    double intake;
    double shoulder;
    double wrist;
  };
  SuperstructureGoal superstructure_goal_;

  void MoveSuperstructure(double intake, double shoulder, double wrist,
                          const ProfileParameters intake_params,
                          const ProfileParameters shoulder_params,
                          const ProfileParameters wrist_params,
                          bool traverse_up, double roller_power);
  void WaitForSuperstructure();
  void WaitForSuperstructureProfile();
  void WaitForSuperstructureLow();
  void WaitForIntake();
  bool IntakeDone();
  bool WaitForDriveProfileDone();

  void FrontLongShot();
  void FrontMiddleShot();
  void BackLongShot();
  void BackLongShotTwoBall();
  void BackLongShotTwoBallFinish();
  void BackLongShotLowBarTwoBall();
  void BackMiddleShot();
  void WaitForBall();
  void TuckArm(bool arm_down, bool traverse_down);
  void OpenShooter();
  void CloseShooter();
  void CloseIfBall();
  bool SuperstructureProfileDone();
  bool SuperstructureDone();
  void TippyDrive(double goal_distance, double tip_distance, double below,
                  double above);

  void DoFullShot();
  void LowBarDrive();
  // Drive to the middle spot over the middle position.  Designed for the rock
  // wall, rough terain, or ramparts.
  void MiddleDrive();

  void OneFromMiddleDrive(bool left);
  void TwoFromMiddleDrive();

  double shooter_speed_ = 0.0;
  void SetShooterSpeed(double speed);
  void WaitForShooterSpeed();
  void Shoot();

  void AlignWithVisionGoal();
  void WaitForAlignedWithVision(aos::time::Time align_duration);

  void TwoBallAuto();

  ::std::unique_ptr<actors::VisionAlignAction> vision_action_;
};

typedef ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>
    AutonomousAction;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
