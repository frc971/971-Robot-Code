#ifndef Y2016_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

#include "y2016_bot3/actors/autonomous_action.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2016_bot3 {
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
                  ::frc971::ProfileParameters linear);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();
  void WaitForBallOrDriveDone();

  void StealAndMoveOverBy(double distance);

  // Returns true if the drive has finished.
  bool IsDriveDone();

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
  struct IntakeGoal {
    double intake;
  };
  IntakeGoal intake_goal_;

  void MoveIntake(double intake, const ProfileParameters intake_params,
                  bool traverse_up, double roller_power);
  void WaitForIntakeProfile();
  void WaitForIntakeLow();
  void WaitForIntake();
  bool IntakeDone();
  bool WaitForDriveProfileDone();

  void WaitForBall();

  void LowBarDrive();
  // Drive to the middle spot over the middle position.  Designed for the rock
  // wall, rough terain, or ramparts.
  void MiddleDrive();

  void OneFromMiddleDrive(bool left);
  void TwoFromMiddleDrive();
};

typedef ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>
    AutonomousAction;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016_bot3::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2016_bot3

#endif  // Y2016_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
