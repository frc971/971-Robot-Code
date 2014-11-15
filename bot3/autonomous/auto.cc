#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"

#include "bot3/actions/drivetrain_action.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/drivetrain/drivetrain_constants.h"
#include "bot3/control_loops/rollers/rollers.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/drivetrain_action.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/queues/other_sensors.q.h"

using ::aos::time::Time;

namespace bot3 {
namespace autonomous {

namespace time = ::aos::time;

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::frc971::autonomous::autonomous.FetchLatest();
  bool ans = !::frc971::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void StopDrivetrain() {
  LOG(INFO, "Stopping the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .quickturn(false)
      .Send();
}

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
}

void WaitUntilDoneOrCanceled(::frc971::Action *action) {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoop10MS(5000);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

::std::unique_ptr<::frc971::TypedAction
    < ::frc971::actions::DrivetrainActionQueueGroup>>
SetDriveGoal(double distance, bool slow_acceleration,
             double maximum_velocity = 1.7, double theta = 0) {
  LOG(INFO, "Driving to %f\n", distance);
  auto drivetrain_action = actions::MakeDrivetrainAction();
  drivetrain_action->GetGoal()->left_initial_position = left_initial_position;
  drivetrain_action->GetGoal()->right_initial_position = right_initial_position;
  drivetrain_action->GetGoal()->y_offset = distance;
  drivetrain_action->GetGoal()->theta_offset = theta;
  drivetrain_action->GetGoal()->maximum_velocity = maximum_velocity;
  drivetrain_action->GetGoal()->maximum_acceleration =
      slow_acceleration ? 2.5 : 3.0;
  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * control_loops::kBot3TurnWidth / 2.0;
  right_initial_position +=
      distance + theta * control_loops::kBot3TurnWidth / 2.0;
  return ::std::move(drivetrain_action);
}

void InitializeEncoders() {
  control_loops::drivetrain.status.FetchLatest();
  while (!control_loops::drivetrain.status.get()) {
    LOG(WARNING,
        "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain.status.FetchNextBlocking();
  }
  left_initial_position =
    control_loops::drivetrain.status->filtered_left_position;
  right_initial_position =
    control_loops::drivetrain.status->filtered_right_position;

}

void StopRollers() {
  control_loops::rollers.goal.MakeWithBuilder()
    .intake(0)
    .low_spit(0)
    .human_player(false)
    .Send();
}

void SpitBallFront() {
  control_loops::rollers.goal.MakeWithBuilder()
      .intake(0)
      .low_spit(1)
      .human_player(false)
      .Send();
  time::SleepFor(time::Time::InSeconds(1));
  StopRollers();
}

void IntakeBallBack() {
  control_loops::rollers.goal.MakeWithBuilder()
    .intake(-1)
    .low_spit(0)
    .human_player(false)
    .Send();
  time::SleepFor(time::Time::InSeconds(1.5));
  StopRollers();
}

void ScoreBall(const double distance, const double velocity) {
  // Drive to the goal, score, and drive back.
  {
    // Drive forward.
    auto drivetrain_action = SetDriveGoal(distance,
                                          false, velocity);
    LOG(INFO, "Waiting until drivetrain is finished.\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    time::SleepFor(time::Time::InSeconds(0.5));
    if (ShouldExitAuto()) return;
  }
  {
    LOG(INFO, "Spitting ball.\n");
    SpitBallFront();
    time::SleepFor(time::Time::InSeconds(0.5));
    if (ShouldExitAuto()) return;
  }
  {
    // Drive back.
    LOG(INFO, "Driving back.\n");
    auto drivetrain_action = SetDriveGoal(-distance,
                                          false, velocity);
    LOG(INFO, "Waiting until drivetrain is finished.\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  }
}

void HandleAuto() {
  constexpr double kDriveDistance = 4.86;
  constexpr double kAutoVelocity = 2.5;

  if (ShouldExitAuto()) return;
  ResetDrivetrain();
  InitializeEncoders();
  if (ShouldExitAuto()) return;

  ScoreBall(kDriveDistance, kAutoVelocity);
}

}  // namespace autonomous
}  // namespace bot3
