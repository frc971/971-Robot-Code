#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network/team_number.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/shoot_action.h"
#include "frc971/actions/drivetrain_action.h"
#include "frc971/queues/other_sensors.q.h"
#include "frc971/queues/hot_goal.q.h"

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

void DriveSpin(double radians) {
  LOG(INFO, "going to spin %f\n", radians);

  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  // in drivetrain "meters"
  const double kRobotWidth = 0.4544;

  profile.set_maximum_acceleration(1.5);
  profile.set_maximum_velocity(0.8);

  const double side_offset = kRobotWidth * radians / 2.0;

  while (true) {
    ::aos::time::PhasedLoop10MS(5000);      // wait until next 10ms tick
    driveTrainState = profile.Update(side_offset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - side_offset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_initial_position - driveTrainState(0, 0),
        right_initial_position + driveTrainState(0, 0));
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(left_initial_position - driveTrainState(0, 0))
        .right_goal(right_initial_position + driveTrainState(0, 0))
        .left_velocity_goal(-driveTrainState(1, 0))
        .right_velocity_goal(driveTrainState(1, 0))
        .Send();
  }
  left_initial_position -= side_offset;
  right_initial_position += side_offset;
  LOG(INFO, "Done moving\n");
}

void PositionClawVertically(double intake_power = 0.0, double centering_power = 0.0) {
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(0.0)
           .separation_angle(0.0)
           .intake(intake_power)
           .centering(centering_power)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void PositionClawBackIntake() {
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(-2.273474)
           .separation_angle(0.0)
           .intake(12.0)
           .centering(12.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void PositionClawForShot() {
  // Turn the claw on, keep it straight up until the ball has been grabbed.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(0.86)
           .separation_angle(0.10)
           .intake(4.0)
           .centering(1.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void SetShotPower(double power) {
  LOG(INFO, "Setting shot power to %f\n", power);
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
           .shot_power(power)
           .shot_requested(false)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
  }
}

void WaitUntilDoneOrCanceled(Action *action) {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoop10MS(5000);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

void Shoot() {
  // Shoot.
  auto shoot_action = actions::MakeShootAction();
  shoot_action->Start();
  WaitUntilDoneOrCanceled(shoot_action.get());
}

::std::unique_ptr<TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
SetDriveGoal(double distance, double maximum_velocity = 1.7, double theta = 0) {
  LOG(INFO, "Driving to %f\n", distance);
  auto drivetrain_action = actions::MakeDrivetrainAction();
  drivetrain_action->GetGoal()->left_initial_position = left_initial_position;
  drivetrain_action->GetGoal()->right_initial_position = right_initial_position;
  drivetrain_action->GetGoal()->y_offset = distance;
  drivetrain_action->GetGoal()->theta_offset = theta;
  drivetrain_action->GetGoal()->maximum_velocity = maximum_velocity;
  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * constants::GetValues().turn_width / 2.0;
  right_initial_position +=
      distance + theta * constants::GetValues().turn_width / 2. -
      theta * constants::GetValues().turn_width / 2.00;
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

void WaitUntilClawDone() {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoop10MS(5000);
    control_loops::claw_queue_group.status.FetchLatest();
    control_loops::claw_queue_group.goal.FetchLatest();
    if (ShouldExitAuto()) {
      return;
    }
    if (control_loops::claw_queue_group.status.get() == nullptr ||
        control_loops::claw_queue_group.goal.get() == nullptr) {
      continue;
    }
    bool ans =
        control_loops::claw_queue_group.status->zeroed &&
        (::std::abs(control_loops::claw_queue_group.status->bottom_velocity) <
         1.0) &&
        (::std::abs(control_loops::claw_queue_group.status->bottom -
                    control_loops::claw_queue_group.goal->bottom_angle) <
         0.10) &&
        (::std::abs(control_loops::claw_queue_group.status->separation -
                    control_loops::claw_queue_group.goal->separation_angle) <
         0.4);
    if (ans) {
      return;
    }
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
  LOG(INFO, "running auto %" PRIu8 "\n", auto_version);

  HotGoalDecoder hot_goal_decoder;
  // True for left, false for right.
  bool first_shot_left, second_shot_left_default, second_shot_left;

  ResetDrivetrain();

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
    auto drivetrain_action = SetDriveGoal(-kShootDistance, 2.5);
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
        SetDriveGoal(0, 2, first_shot_left ? kTurnAngle : -kTurnAngle);
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
  }

  // Shoot.
  LOG(INFO, "Shooting at %f\n",
      (::aos::time::Time::Now() - start_time).ToSeconds());
  Shoot();
  time::SleepFor(time::Time::InSeconds(0.05));

  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldExitAuto()) return;
    auto drivetrain_action =
        SetDriveGoal(0, 2, first_shot_left ? -kTurnAngle : kTurnAngle);
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
        SetDriveGoal(kShootDistance + kPickupDistance, 2.5);
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
        SetDriveGoal(-(kShootDistance + kPickupDistance), 2.5);
    time::SleepFor(time::Time::InSeconds(0.3));
    hot_goal_decoder.ResetCounts();
    if (ShouldExitAuto()) return;
    PositionClawForShot();
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitUntilDoneOrCanceled(drivetrain_action.get());
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
        SetDriveGoal(0, 2, second_shot_left ? kTurnAngle : -kTurnAngle);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
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
