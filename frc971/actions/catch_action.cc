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
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(action_q_->goal->catch_angle)
           .separation_angle(kCatchSeparation)
           .intake(kCatchIntake)
           .centering(kCatchCentering)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }
  LOG(INFO, "Waiting for the claw to be ready\n");

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&CatchAction::DoneSetupCatch, this))) return;
  LOG(INFO, "Waiting for the sonar\n");

  // wait for the sonar to trigger
  if (WaitUntil(::std::bind(&CatchAction::DoneFoundSonar, this))) return;

  LOG(INFO, "Closing the claw\n");

  // close the claw
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(action_q_->goal->catch_angle + kCatchSeparation / 2.0)
           .separation_angle(0.0)
           .intake(kCatchIntake)
           .centering(kCatchCentering)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // claw now closed
  if (WaitUntil(::std::bind(&CatchAction::DoneClawWithBall, this))) return;

  return;
}


/*bool CatchAction::DoneBallIn() {
  if (!sensors::othersensors.FetchLatest()) {
    sensors::othersensors.FetchNextBlocking();
  }
  if (sensors::othersensors->travis_hall_effect_distance > 0.005) {
    LOG(INFO, "Ball in at %.2f.\n",
        sensors::othersensors->travis_hall_effect_distance);
    return true;
  }
  return false;
}*/

bool CatchAction::DoneClawWithBall() {
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
    control_loops::claw_queue_group.status.FetchNextBlocking();
  }

  bool ans =
      control_loops::claw_queue_group.status->zeroed &&
      (::std::abs(control_loops::claw_queue_group.status->bottom_velocity) <
       0.5) &&
      (::std::abs(control_loops::claw_queue_group.status->bottom -
                  control_loops::claw_queue_group.goal->bottom_angle) < 0.10) &&
      (::std::abs(control_loops::claw_queue_group.status->separation -
                  control_loops::claw_queue_group.goal->separation_angle) <
       0.4);

  if (!ans) {
    LOG(INFO,
        "Claw is ready %d zeroed %d bottom_velocity %f bottom %f sep %f\n", ans,
        control_loops::claw_queue_group.status->zeroed,
        ::std::abs(control_loops::claw_queue_group.status->bottom_velocity),
        ::std::abs(control_loops::claw_queue_group.status->bottom -
                   control_loops::claw_queue_group.goal->bottom_angle),
        ::std::abs(control_loops::claw_queue_group.status->separation -
                   control_loops::claw_queue_group.goal->separation_angle));
  }
  return ans;
}

bool CatchAction::DoneFoundSonar() {
  if (!sensors::othersensors.FetchLatest()) {
    sensors::othersensors.FetchNextBlocking();
  }
  LOG(INFO, "Sonar at %.2f.\n", sensors::othersensors->sonar_distance);
  if (sensors::othersensors->sonar_distance > 0.3 &&
      sensors::othersensors->sonar_distance < kSonarTriggerDist) {
    return true;
  }
  return false;
}

bool CatchAction::DoneSetupCatch() {
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
    control_loops::claw_queue_group.status.FetchNextBlocking();
  }

  // Make sure that the shooter and claw has reached the necessary state.
  // Check the current positions of the various mechanisms to make sure that we
  // avoid race conditions where we send it a new goal but it still thinks that
  // it has the old goal and thinks that it is already done.
  bool claw_angle_correct =
      ::std::abs(control_loops::claw_queue_group.status->bottom -
                 action_q_->goal->catch_angle) < 0.15;
  bool open_enough =
      control_loops::claw_queue_group.status->separation > kCatchMinSeparation;

  if (claw_angle_correct && open_enough) {
    LOG(INFO, "Claw ready for catching.\n");
    return true;
  }

  return false;
}

}  // namespace actions
}  // namespace frc971

