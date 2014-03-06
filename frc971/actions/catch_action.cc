#include <functional>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"

#include "frc971/actions/catch_action.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/queues/othersensors.q.h"

namespace frc971 {
namespace actions {

CatchAction::CatchAction(actions::CatchActionGroup* s)
    : actions::ActionBase<actions::CatchActionGroup>(s) {}

void CatchAction::RunAction() {

  // Set claw angle.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          catch_action.goal->catch_angle)
          .separation_angle(kCatchSeparation).intake(kCatchIntake)
          .centering(kCatchCentering).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&CatchAction::DoneSetupCatch, this))) return;

  // Set claw angle.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          catch_action.goal->catch_angle).separation_angle(kCatchSeparation)
          .intake(kCatchIntake).centering(kCatchCentering).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for the sonar to trigger
  if (WaitUntil(::std::bind(&CatchAction::DoneFoundSonar, this))) return;

  // close the claw
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          kFinishAngle).separation_angle(0.0).intake(kCatchIntake)
          .centering(kCatchCentering).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // claw now closed
  if (WaitUntil(::std::bind(&CatchAction::DoneClawWithBall, this))) return;
  // ball is fully in
  if (WaitUntil(::std::bind(&CatchAction::DoneBallIn, this))) return;

  // head to a finshed pose
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          kFinishAngle)
          .separation_angle(0.0).intake(0.0).centering(0.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // thats it
  if (WaitUntil(::std::bind(&CatchAction::DoneClawWithBall, this))) return;

  // done with action
  return;
}


bool CatchAction::DoneBallIn() {
  if (!sensors::othersensors.FetchLatest()) {
  	sensors::othersensors.FetchNextBlocking();
  }
  if (sensors::othersensors->travis_hall_effect_distance > 0.005) {
    LOG(INFO, "Ball in at %.2f.\n",
        sensors::othersensors->travis_hall_effect_distance);
  	return true;
  }
  return false;
}

bool CatchAction::DoneClawWithBall() {
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

bool CatchAction::DoneFoundSonar() {
  if (!sensors::othersensors.FetchLatest()) {
  	sensors::othersensors.FetchNextBlocking();
  }
  if (sensors::othersensors->sonar_distance > 0.3 &&
      sensors::othersensors->sonar_distance < kSonarTriggerDist) {
    LOG(INFO, "Hit Sonar at %.2f.\n", sensors::othersensors->sonar_distance);
  	return true;
  }
  return false;
}

bool CatchAction::DoneSetupCatch() {
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
  	control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  if (!control_loops::claw_queue_group.goal.FetchLatest()) {
    LOG(ERROR, "Failed to fetch claw goal.\n");
  }
  // Make sure that the shooter and claw has reached the necessary state.
  // Check the current positions of the various mechanisms to make sure that we
  // avoid race conditions where we send it a new goal but it still thinks that
  // it has the old goal and thinks that it is already done.
  bool claw_angle_correct =
      ::std::abs(control_loops::claw_queue_group.status->bottom -
                 control_loops::claw_queue_group.goal->bottom_angle) <
      0.005;

  if (control_loops::claw_queue_group.status->done && claw_angle_correct) {
    LOG(INFO, "Claw ready for catching.\n");
    return true;
  }

  return false;
}

}  // namespace actions
}  // namespace frc971

