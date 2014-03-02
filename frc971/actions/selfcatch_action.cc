#include <functional>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"
#include "frc971/constants.h"

#include "frc971/actions/selfcatch_action.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/queues/othersensors.q.h"

namespace frc971 {
namespace actions {

SelfCatchAction::SelfCatchAction(actions::SelfCatchActionGroup* s)
    : actions::ActionBase<actions::SelfCatchActionGroup>(s) {}

double SelfCatchAction::SpeedToAngleOffset(double speed) {
  const frc971::constants::Values& values = frc971::constants::GetValues();
  // scale speed to a [0.0-1.0] on something close to the max
  return (speed / values.drivetrain_max_speed) *
         SelfCatchAction::kSpeedOffsetRadians;
}

void SelfCatchAction::RunAction() {
  const frc971::constants::Values& values = frc971::constants::GetValues();

  // Set shot power to established constant
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          kShotPower)
          .shot_requested(false).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(ERROR, "Failed to send the shoot action\n");
    return;
  }

  // Set claw angle to account for velocity
  control_loops::drivetrain.status.FetchLatest();
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          0.0 +
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(0.0).intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneSetupShot, this))) return;

  // Open up the claw in preparation for shooting.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          0.0 +
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(values.shooter_action.claw_separation_goal)
          .intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for the claw to open up a little before we shoot
  if (WaitUntil(::std::bind(&SelfCatchAction::DonePreShotOpen, this))) return;

  // Make sure that we have the latest shooter status.
  control_loops::shooter_queue_group.status.FetchLatest();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  previous_shots_ = control_loops::shooter_queue_group.status->shots;
  // Shoot!
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          kShotPower)
          .shot_requested(true).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }

  // wait for record of shot having been fired
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneShot, this))) return;
  
  // Set claw angle to account for velocity note this is negative
  // since we will be catching from behind
  control_loops::drivetrain.status.FetchLatest();
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          0.0 -
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(kCatchSeperation).intake(kCatchIntake)
          .centering(kCatchCentering).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for the sonar to trigger
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneFoundSonar, this))) return;

  // close the claw
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          kFinishAngle)
          .separation_angle(0.0).intake(kCatchIntake).centering(kCatchCentering)
          .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // claw now closed
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneClawWithBall, this))) return;
  // ball is fully in
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneBallIn, this))) return;

  // head to a finshed pose
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          kFinishAngle)
          .separation_angle(kFinishAngle).intake(0.0).centering(0.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // thats it
  if (WaitUntil(::std::bind(&SelfCatchAction::DoneClawWithBall, this))) return;

  // done with action
  return;
}


bool SelfCatchAction::DoneBallIn() {
  if (!queues::othersensors.FetchLatest()) {
  	queues::othersensors.FetchNextBlocking();
  }
  if (queues::othersensors->travis_hall_effect_distance > 0.005)
    LOG(INFO, "Ball in at %.2f.\n",
        queues::othersensors->travis_hall_effect_distance);
  	return true;
  }
  return false;
}

bool SelfCatchAction::DoneClawWithBall() {
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
  	control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (control_loops::claw_queue_group.status->done_with_ball) {
    LOG(INFO, "Claw at goal.\n");
    return true;
  }
  return false;
}

bool SelfCatchAction::DoneFoundSonar() {
  if (!queues::othersensors.FetchLatest()) {
  	queues::othersensors.FetchNextBlocking();
  }
  if (queues::othersensors->sonar_distance > 0.3 &&
      queues::othersensors->sonar_distance < kSonarTriggerDist) {
    LOG(INFO, "Hit Sonar at %.2f.\n", queues::othersensors->sonar_distance);
  	return true;
  }
  return false;
}

bool SelfCatchAction::DoneSetupShot() {
  if (!control_loops::shooter_queue_group.status.FetchLatest()) {
  	control_loops::shooter_queue_group.status.FetchNextBlocking();
  }
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
  	control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (control_loops::shooter_queue_group.status->ready &&
      control_loops::claw_queue_group.status->done_with_ball) {
    LOG(INFO, "Claw and Shooter ready for shooting.\n");
    // TODO(james): Get realer numbers for shooter_action.
    return true;
  }

  // update the claw position to track velocity
  // TODO(ben): the claw may never reach the goal if the velocity is
  // continually changing, we will need testing to see
  control_loops::drivetrain.status.FetchLatest();
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle +
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(0.0).intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    abort_ = true;
    return true;
  } else {
    LOG(INFO, "Updating claw angle for velocity offset(%.4f).\n",
        SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed));
  }
  return false;
}

bool SelfCatchAction::DonePreShotOpen() {
  const frc971::constants::Values& values = frc971::constants::GetValues();
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
  	control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  if (control_loops::claw_queue_group.status->separation >
      values.shooter_action.claw_shooting_separation) {
    LOG(INFO, "Opened up enough to shoot.\n");
    return true;
  }
  return false;
}

bool SelfCatchAction::DoneShot() {
  if (!control_loops::shooter_queue_group.status.FetchLatest()) {
  	control_loops::shooter_queue_group.status.FetchNextBlocking();
  }
  if (control_loops::shooter_queue_group.status->shots > previous_shots_) {
    LOG(INFO, "Shot succeeded!\n");
    return true;
  }
  return false;
}

}  // namespace actions
}  // namespace frc971

