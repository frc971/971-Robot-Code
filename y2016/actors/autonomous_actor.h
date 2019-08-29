#ifndef Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/event_loop.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2016/actors/vision_align_actor.h"
#include "y2016/control_loops/shooter/shooter_goal_generated.h"
#include "y2016/control_loops/shooter/shooter_status_generated.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"
#include "y2016/queues/ball_detector_generated.h"

namespace y2016 {
namespace actors {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

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
                          const frc971::ProfileParametersT intake_params,
                          const frc971::ProfileParametersT shoulder_params,
                          const frc971::ProfileParametersT wrist_params,
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

  actors::VisionAlignActor::Factory vision_align_actor_factory_;
  ::std::unique_ptr<::aos::common::actions::Action> vision_action_;

  ::aos::Fetcher<::y2016::vision::VisionStatus> vision_status_fetcher_;
  ::aos::Fetcher<::y2016::sensors::BallDetector> ball_detector_fetcher_;
  ::aos::Sender<::y2016::control_loops::shooter::Goal> shooter_goal_sender_;
  ::aos::Fetcher<::y2016::control_loops::shooter::Status>
      shooter_status_fetcher_;
  ::aos::Fetcher<::y2016::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  ::aos::Sender<::y2016::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
};

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
