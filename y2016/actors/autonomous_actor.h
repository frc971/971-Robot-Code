#ifndef Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2016/actors/vision_align_actor.h"

namespace y2016 {
namespace actors {
using ::frc971::ProfileParameters;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::frc971::autonomous::AutonomousActionQueueGroup *s);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams &params) override;

 private:
  void WaitForBallOrDriveDone();

  void StealAndMoveOverBy(double distance);

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
  void WaitForAlignedWithVision(::std::chrono::nanoseconds align_duration);

  void TwoBallAuto();

  ::std::unique_ptr<actors::VisionAlignAction> vision_action_;
};

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
