#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/autonomous/auto.q.h"
#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/shoot_action.h"
#include "frc971/actions/drivetrain_action.h"
#include "frc971/queues/other_sensors.q.h"
#include "y2014/queues/hot_goal.q.h"

using ::aos::time::Time;

namespace frc971 {
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
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
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
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
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

void WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoopXMS(5, 2500);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

void StepDrive(double distance, double theta) {
  double left_goal = (left_initial_position + distance -
                      theta * constants::GetValues().turn_width / 2.0);
  double right_goal = (right_initial_position + distance +
                       theta * constants::GetValues().turn_width / 2.0);
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(left_goal)
      .right_goal(right_goal)
      .left_velocity_goal(0.0)
      .right_velocity_goal(0.0)
      .Send();
  left_initial_position = left_goal;
  right_initial_position = right_goal;
}

void WaitUntilNear(double distance) {
  while (true) {
    if (ShouldExitAuto()) return;
    control_loops::drivetrain_queue.status.FetchAnother();
    double left_error = ::std::abs(
        left_initial_position -
        control_loops::drivetrain_queue.status->filtered_left_position);
    double right_error = ::std::abs(
        right_initial_position -
        control_loops::drivetrain_queue.status->filtered_right_position);
    const double kPositionThreshold = 0.05 + distance;
    if (right_error < kPositionThreshold && left_error < kPositionThreshold) {
      LOG(INFO, "At the goal\n");
      return;
    }
  }
}

const ProfileParams kFastDrive = {2.0, 3.5};
const ProfileParams kFastKnockDrive = {2.0, 3.0};
const ProfileParams kStackingSecondDrive = {2.0, 1.5};
const ProfileParams kFastTurn = {3.0, 10.0};
const ProfileParams kStackingFirstTurn = {1.0, 1.0};
const ProfileParams kStackingSecondTurn = {2.0, 6.0};
const ProfileParams kComboTurn = {1.2, 8.0};
const ProfileParams kRaceTurn = {4.0, 10.0};
const ProfileParams kRaceDrive = {2.0, 2.0};
const ProfileParams kRaceBackupDrive = {2.0, 5.0};

::std::unique_ptr<::frc971::actors::DrivetrainAction> SetDriveGoal(
    double distance, const ProfileParams drive_params, double theta = 0,
    const ProfileParams &turn_params = kFastTurn) {
  LOG(INFO, "Driving to %f\n", distance);

  ::frc971::actors::DrivetrainActionParams params;
  params.left_initial_position = left_initial_position;
  params.right_initial_position = right_initial_position;
  params.y_offset = distance;
  params.theta_offset = theta;
  params.maximum_turn_acceleration = turn_params.acceleration;
  params.maximum_turn_velocity = turn_params.velocity;
  params.maximum_velocity = drive_params.velocity;
  params.maximum_acceleration = drive_params.acceleration;
  auto drivetrain_action = actors::MakeDrivetrainAction(params);
  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * constants::GetValues().turn_width / 2.0;
  right_initial_position +=
      distance + theta * constants::GetValues().turn_width / 2.0;
  return ::std::move(drivetrain_action);
}

void Shoot() {
  // Shoot.
  auto shoot_action = actions::MakeShootAction();
  shoot_action->Start();
  WaitUntilDoneOrCanceled(shoot_action.get());
}

void InitializeEncoders() {
  control_loops::drivetrain_queue.status.FetchAnother();
  left_initial_position =
      control_loops::drivetrain_queue.status->filtered_left_position;
  right_initial_position =
      control_loops::drivetrain_queue.status->filtered_right_position;
}

void WaitUntilClawDone() {
  while (true) {
    // Poll the running bit and auto done bits.
    // TODO(sensors): Fix this time.
    ::aos::time::PhasedLoop10MS(5000);
    control_loops::claw_queue.status.FetchLatest();
    control_loops::claw_queue.goal.FetchLatest();
    if (ShouldExitAuto()) {
      return;
    }
    if (control_loops::claw_queue.status.get() == nullptr ||
        control_loops::claw_queue.goal.get() == nullptr) {
      continue;
    }
    bool ans =
        control_loops::claw_queue.status->zeroed &&
        (::std::abs(control_loops::claw_queue.status->bottom_velocity) <
         1.0) &&
        (::std::abs(control_loops::claw_queue.status->bottom -
                    control_loops::claw_queue.goal->bottom_angle) <
         0.10) &&
        (::std::abs(control_loops::claw_queue.status->separation -
                    control_loops::claw_queue.goal->separation_angle) <
         0.4);
    if (ans) {
      return;
    }
    if (ShouldExitAuto()) return;
  }
}

class HotGoalDecoder {
 public:
  HotGoalDecoder() {
    ResetCounts();
  }

  void ResetCounts() {
    hot_goal.FetchLatest();
    if (hot_goal.get()) {
      start_counts_ = *hot_goal;
      LOG_STRUCT(INFO, "counts reset to", start_counts_);
      start_counts_valid_ = true;
    } else {
      LOG(WARNING, "no hot goal message. ignoring\n");
      start_counts_valid_ = false;
    }
  }

  void Update(bool block = false) {
    if (block) {
      hot_goal.FetchAnother();
    } else {
      hot_goal.FetchLatest();
    }
    if (hot_goal.get()) LOG_STRUCT(INFO, "new counts", *hot_goal);
  }

  bool left_triggered() const {
    if (!start_counts_valid_ || !hot_goal.get()) return false;
    return (hot_goal->left_count - start_counts_.left_count) > kThreshold;
  }

  bool right_triggered() const {
    if (!start_counts_valid_ || !hot_goal.get()) return false;
    return (hot_goal->right_count - start_counts_.right_count) > kThreshold;
  }

  bool is_left() const {
    if (!start_counts_valid_ || !hot_goal.get()) return false;
    const uint64_t left_difference =
        hot_goal->left_count - start_counts_.left_count;
    const uint64_t right_difference =
        hot_goal->right_count - start_counts_.right_count;
    if (left_difference > kThreshold) {
      if (right_difference > kThreshold) {
        // We've seen a lot of both, so pick the one we've seen the most of.
        return left_difference > right_difference;
      } else {
        // We've seen enough left but not enough right, so go with it.
        return true;
      }
    } else {
      // We haven't seen enough left, so it's not left.
      return false;
    }
  }

  bool is_right() const {
    if (!start_counts_valid_ || !hot_goal.get()) return false;
    const uint64_t left_difference =
        hot_goal->left_count - start_counts_.left_count;
    const uint64_t right_difference =
        hot_goal->right_count - start_counts_.right_count;
    if (right_difference > kThreshold) {
      if (left_difference > kThreshold) {
        // We've seen a lot of both, so pick the one we've seen the most of.
        return right_difference > left_difference;
      } else {
        // We've seen enough right but not enough left, so go with it.
        return true;
      }
    } else {
      // We haven't seen enough right, so it's not right.
      return false;
    }
  }

 private:
  static const uint64_t kThreshold = 5;

  ::frc971::HotGoal start_counts_;
  bool start_counts_valid_;
};

void HandleAuto() {
  enum class AutoVersion : uint8_t {
    kStraight,
    kDoubleHot,
    kSingleHot,
  };

  // The front of the robot is 1.854 meters from the wall
  static const double kShootDistance = 3.15;
  static const double kPickupDistance = 0.5;
  static const double kTurnAngle = 0.3;

  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Handling auto mode\n");

  AutoVersion auto_version;
  ::frc971::sensors::auto_mode.FetchLatest();
  if (!::frc971::sensors::auto_mode.get()) {
    LOG(WARNING, "not sure which auto mode to use\n");
    auto_version = AutoVersion::kStraight;
  } else {
    static const double kSelectorMin = 0.2, kSelectorMax = 4.4;

    const double kSelectorStep = (kSelectorMax - kSelectorMin) / 3.0;
    if (::frc971::sensors::auto_mode->voltage < kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kSingleHot;
    } else if (::frc971::sensors::auto_mode->voltage <
               2 * kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kStraight;
    } else {
      auto_version = AutoVersion::kDoubleHot;
    }
  }
  LOG(INFO, "running auto %" PRIu8 "\n", static_cast<uint8_t>(auto_version));

  const bool drive_slow_acceleration = auto_version == AutoVersion::kStraight;

  HotGoalDecoder hot_goal_decoder;
  // True for left, false for right.
  bool first_shot_left, second_shot_left_default, second_shot_left;

  ResetDrivetrain();

  time::SleepFor(time::Time::InSeconds(0.1));
  if (ShouldExitAuto()) return;
  InitializeEncoders();

  // Turn the claw on, keep it straight up until the ball has been grabbed.
  LOG(INFO, "Claw going up at %f\n",
      (::aos::time::Time::Now() - start_time).ToSeconds());
  PositionClawVertically(12.0, 4.0);
  SetShotPower(115.0);

  // Wait for the ball to enter the claw.
  time::SleepFor(time::Time::InSeconds(0.25));
  if (ShouldExitAuto()) return;
  LOG(INFO, "Readying claw for shot at %f\n",
      (::aos::time::Time::Now() - start_time).ToSeconds());

  {
    if (ShouldExitAuto()) return;
    // Drive to the goal.
    auto drivetrain_action = SetDriveGoal(-kShootDistance,
                                          drive_slow_acceleration, 2.5);
    time::SleepFor(time::Time::InSeconds(0.75));
    PositionClawForShot();
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  }

  hot_goal_decoder.Update();
  if (hot_goal_decoder.is_left()) {
    LOG(INFO, "first shot left\n");
    first_shot_left = true;
    second_shot_left_default = false;
  } else if (hot_goal_decoder.is_right()) {
    LOG(INFO, "first shot right\n");
    first_shot_left = false;
    second_shot_left_default = true;
  } else {
    LOG(INFO, "first shot defaulting left\n");
    first_shot_left = true;
    second_shot_left_default = true;
  }
  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldExitAuto()) return;
    auto drivetrain_action =
        SetDriveGoal(0, drive_slow_acceleration, 2,
                     first_shot_left ? kTurnAngle : -kTurnAngle);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  } else if (auto_version == AutoVersion::kSingleHot) {
    do {
      // TODO(brians): Wait for next message with timeout or something.
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.003));
      hot_goal_decoder.Update(false);
      if (ShouldExitAuto()) return;
    } while (!hot_goal_decoder.left_triggered() &&
             (::aos::time::Time::Now() - start_time) <
                 ::aos::time::Time::InSeconds(9));
  } else if (auto_version == AutoVersion::kStraight) {
    time::SleepFor(time::Time::InSeconds(0.4));
  }

  // Shoot.
  LOG(INFO, "Shooting at %f\n",
      (::aos::time::Time::Now() - start_time).ToSeconds());
  Shoot();
  time::SleepFor(time::Time::InSeconds(0.05));

  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldExitAuto()) return;
    auto drivetrain_action =
        SetDriveGoal(0, drive_slow_acceleration, 2,
                     first_shot_left ? -kTurnAngle : kTurnAngle);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  } else if (auto_version == AutoVersion::kSingleHot) {
    LOG(INFO, "auto done at %f\n",
        (::aos::time::Time::Now() - start_time).ToSeconds());
    PositionClawVertically(0.0, 0.0);
    return;
  }

  {
    if (ShouldExitAuto()) return;
    // Intake the new ball.
    LOG(INFO, "Claw ready for intake at %f\n",
        (::aos::time::Time::Now() - start_time).ToSeconds());
    PositionClawBackIntake();
    auto drivetrain_action =
        SetDriveGoal(kShootDistance + kPickupDistance,
                     drive_slow_acceleration, 2.5);
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
    LOG(INFO, "Wait for the claw at %f\n",
        (::aos::time::Time::Now() - start_time).ToSeconds());
    WaitUntilClawDone();
    if (ShouldExitAuto()) return;
  }

  // Drive back.
  {
    LOG(INFO, "Driving back at %f\n",
        (::aos::time::Time::Now() - start_time).ToSeconds());
    auto drivetrain_action =
        SetDriveGoal(-(kShootDistance + kPickupDistance),
                     drive_slow_acceleration, 2.5);
    time::SleepFor(time::Time::InSeconds(0.3));
    hot_goal_decoder.ResetCounts();
    if (ShouldExitAuto()) return;
    PositionClawUpClosed();
    WaitUntilClawDone();
    if (ShouldExitAuto()) return;
    PositionClawForShot();
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
    WaitUntilClawDone();
    if (ShouldExitAuto()) return;
  }

  hot_goal_decoder.Update();
  if (hot_goal_decoder.is_left()) {
    LOG(INFO, "second shot left\n");
    second_shot_left = true;
  } else if (hot_goal_decoder.is_right()) {
    LOG(INFO, "second shot right\n");
    second_shot_left = false;
  } else {
    LOG(INFO, "second shot defaulting %s\n",
        second_shot_left_default ? "left" : "right");
    second_shot_left = second_shot_left_default;
  }
  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldExitAuto()) return;
    auto drivetrain_action =
        SetDriveGoal(0, drive_slow_acceleration, 2,
                     second_shot_left ? kTurnAngle : -kTurnAngle);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  } else if (auto_version == AutoVersion::kStraight) {
    time::SleepFor(time::Time::InSeconds(0.4));
  }

  LOG(INFO, "Shooting at %f\n",
      (::aos::time::Time::Now() - start_time).ToSeconds());
  // Shoot
  Shoot();
  if (ShouldExitAuto()) return;

  // Get ready to zero when we come back up.
  time::SleepFor(time::Time::InSeconds(0.05));
  PositionClawVertically(0.0, 0.0);
}

}  // namespace autonomous
}  // namespace frc971
