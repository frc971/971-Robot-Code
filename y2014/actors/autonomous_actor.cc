#include "y2014/actors/autonomous_actor.h"

#include <stdio.h>

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/actors/shoot_actor.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/drivetrain/drivetrain_base.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/queues/auto_mode.q.h"
#include "y2014/queues/hot_goal.q.h"

namespace y2014 {
namespace actors {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;
using ::aos::monotonic_clock;
using ::frc971::ProfileParameters;

AutonomousActor::AutonomousActor(
    ::aos::EventLoop *event_loop,
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, s, control_loops::GetDrivetrainConfig()) {}

void AutonomousActor::PositionClawVertically(double intake_power,
                                             double centering_power) {
  if (!control_loops::claw_queue.goal.MakeWithBuilder()
           .bottom_angle(0.0)
           .separation_angle(0.0)
           .intake(intake_power)
           .centering(centering_power)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawBackIntake() {
  if (!control_loops::claw_queue.goal.MakeWithBuilder()
           .bottom_angle(-2.273474)
           .separation_angle(0.0)
           .intake(12.0)
           .centering(12.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawUpClosed() {
  // Move the claw to where we're going to shoot from but keep it closed until
  // it gets there.
  if (!control_loops::claw_queue.goal.MakeWithBuilder()
           .bottom_angle(0.86)
           .separation_angle(0.0)
           .intake(4.0)
           .centering(1.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawForShot() {
  if (!control_loops::claw_queue.goal.MakeWithBuilder()
           .bottom_angle(0.86)
           .separation_angle(0.10)
           .intake(4.0)
           .centering(1.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::SetShotPower(double power) {
  LOG(INFO, "Setting shot power to %f\n", power);
  if (!control_loops::shooter_queue.goal.MakeWithBuilder()
           .shot_power(power)
           .shot_requested(false)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
  }
}

const ProfileParameters kFastDrive = {3.0, 2.5};
const ProfileParameters kSlowDrive = {2.5, 2.5};
const ProfileParameters kFastWithBallDrive = {3.0, 2.0};
const ProfileParameters kSlowWithBallDrive = {2.5, 2.0};
const ProfileParameters kFastTurn = {3.0, 10.0};

void AutonomousActor::Shoot() {
  // Shoot.
  auto shoot_action = actors::MakeShootAction();
  shoot_action->Start();
  WaitUntilDoneOrCanceled(::std::move(shoot_action));
}

bool AutonomousActor::WaitUntilClawDone() {
  while (true) {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(10),
                                        ::std::chrono::milliseconds(10) / 2);
    // Poll the running bit and auto done bits.
    phased_loop.SleepUntilNext();
    control_loops::claw_queue.status.FetchLatest();
    control_loops::claw_queue.goal.FetchLatest();
    if (ShouldCancel()) {
      return false;
    }
    if (control_loops::claw_queue.status.get() == nullptr ||
        control_loops::claw_queue.goal.get() == nullptr) {
      continue;
    }
    bool ans =
        control_loops::claw_queue.status->zeroed &&
        (::std::abs(control_loops::claw_queue.status->bottom_velocity) < 1.0) &&
        (::std::abs(control_loops::claw_queue.status->bottom -
                    control_loops::claw_queue.goal->bottom_angle) < 0.10) &&
        (::std::abs(control_loops::claw_queue.status->separation -
                    control_loops::claw_queue.goal->separation_angle) < 0.4);
    if (ans) {
      return true;
    }
  }
}

class HotGoalDecoder {
 public:
  HotGoalDecoder() { ResetCounts(); }

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

  ::y2014::HotGoal start_counts_;
  bool start_counts_valid_;
};

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams & /*params*/) {
  enum class AutoVersion : uint8_t {
    kStraight,
    kDoubleHot,
    kSingleHot,
  };

  // The front of the robot is 1.854 meters from the wall
  static const double kShootDistance = 3.15;
  static const double kPickupDistance = 0.5;
  static const double kTurnAngle = 0.3;

  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Handling auto mode\n");

  AutoVersion auto_version;
  ::y2014::sensors::auto_mode.FetchLatest();
  if (!::y2014::sensors::auto_mode.get()) {
    LOG(WARNING, "not sure which auto mode to use\n");
    auto_version = AutoVersion::kStraight;
  } else {
    static const double kSelectorMin = 0.2, kSelectorMax = 4.4;

    const double kSelectorStep = (kSelectorMax - kSelectorMin) / 3.0;
    if (::y2014::sensors::auto_mode->voltage < kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kSingleHot;
    } else if (::y2014::sensors::auto_mode->voltage <
               2 * kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kStraight;
    } else {
      auto_version = AutoVersion::kDoubleHot;
    }
  }
  LOG(INFO, "running auto %" PRIu8 "\n", static_cast<uint8_t>(auto_version));

  const ProfileParameters &drive_params =
      (auto_version == AutoVersion::kStraight) ? kFastDrive : kSlowDrive;
  const ProfileParameters &drive_with_ball_params =
      (auto_version == AutoVersion::kStraight) ? kFastWithBallDrive
                                               : kSlowWithBallDrive;

  HotGoalDecoder hot_goal_decoder;
  // True for left, false for right.
  bool first_shot_left, second_shot_left_default, second_shot_left;

  Reset();

  // Turn the claw on, keep it straight up until the ball has been grabbed.
  LOG(INFO, "Claw going up at %f\n",
      ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
  PositionClawVertically(12.0, 4.0);
  SetShotPower(115.0);

  // Wait for the ball to enter the claw.
  this_thread::sleep_for(chrono::milliseconds(250));
  if (ShouldCancel()) return true;
  LOG(INFO, "Readying claw for shot at %f\n",
      ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));

  if (ShouldCancel()) return true;
  // Drive to the goal.
  StartDrive(-kShootDistance, 0.0, drive_params, kFastTurn);
  this_thread::sleep_for(chrono::milliseconds(750));
  PositionClawForShot();
  LOG(INFO, "Waiting until drivetrain is finished\n");
  WaitForDriveProfileDone();
  if (ShouldCancel()) return true;

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
    if (ShouldCancel()) return true;
    StartDrive(0, first_shot_left ? kTurnAngle : -kTurnAngle,
                 drive_with_ball_params, kFastTurn);
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
  } else if (auto_version == AutoVersion::kSingleHot) {
    do {
      // TODO(brians): Wait for next message with timeout or something.
      this_thread::sleep_for(chrono::milliseconds(3));
      hot_goal_decoder.Update(false);
      if (ShouldCancel()) return true;
    } while (!hot_goal_decoder.left_triggered() &&
             (monotonic_clock::now() - start_time) < chrono::seconds(9));
  } else if (auto_version == AutoVersion::kStraight) {
    this_thread::sleep_for(chrono::milliseconds(400));
  }

  // Shoot.
  LOG(INFO, "Shooting at %f\n",
      ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
  Shoot();
  this_thread::sleep_for(chrono::milliseconds(50));

  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldCancel()) return true;
    StartDrive(0, first_shot_left ? -kTurnAngle : kTurnAngle,
               drive_with_ball_params, kFastTurn);
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
  } else if (auto_version == AutoVersion::kSingleHot) {
    LOG(INFO, "auto done at %f\n",
        ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
    PositionClawVertically(0.0, 0.0);
    return true;
  }

  {
    if (ShouldCancel()) return true;
    // Intake the new ball.
    LOG(INFO, "Claw ready for intake at %f\n",
        ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
    PositionClawBackIntake();
    StartDrive(kShootDistance + kPickupDistance, 0.0, drive_params, kFastTurn);
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
    LOG(INFO, "Wait for the claw at %f\n",
        ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
    if (!WaitUntilClawDone()) return true;
  }

  // Drive back.
  {
    LOG(INFO, "Driving back at %f\n",
        ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
    StartDrive(-(kShootDistance + kPickupDistance), 0.0, drive_params,
               kFastTurn);
    this_thread::sleep_for(chrono::milliseconds(300));
    hot_goal_decoder.ResetCounts();
    if (ShouldCancel()) return true;
    PositionClawUpClosed();
    if (!WaitUntilClawDone()) return true;
    PositionClawForShot();
    LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
    if (!WaitUntilClawDone()) return true;
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
    if (ShouldCancel()) return true;
    StartDrive(0, second_shot_left ? kTurnAngle : -kTurnAngle, drive_params,
               kFastTurn);
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
  } else if (auto_version == AutoVersion::kStraight) {
    this_thread::sleep_for(chrono::milliseconds(400));
  }

  LOG(INFO, "Shooting at %f\n",
      ::aos::time::DurationInSeconds(monotonic_clock::now() - start_time));
  // Shoot
  Shoot();
  if (ShouldCancel()) return true;

  // Get ready to zero when we come back up.
  this_thread::sleep_for(chrono::milliseconds(50));
  PositionClawVertically(0.0, 0.0);
  return true;
}

}  // namespace actors
}  // namespace y2014
