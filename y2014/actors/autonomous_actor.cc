#include "y2014/actors/autonomous_actor.h"

#include <stdio.h>

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "y2014/actors/shoot_actor.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw_goal_generated.h"
#include "y2014/control_loops/claw/claw_status_generated.h"
#include "y2014/control_loops/drivetrain/drivetrain_base.h"
#include "y2014/control_loops/shooter/shooter_goal_generated.h"
#include "y2014/queues/auto_mode_generated.h"
#include "y2014/queues/hot_goal_generated.h"

namespace y2014 {
namespace actors {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;
using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::GetDrivetrainConfig()),
      auto_mode_fetcher_(
          event_loop->MakeFetcher<::y2014::sensors::AutoMode>("/aos")),
      hot_goal_fetcher_(event_loop->MakeFetcher<::y2014::HotGoal>("/")),
      claw_goal_sender_(
          event_loop->MakeSender<::y2014::control_loops::claw::Goal>("/claw")),
      claw_goal_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::claw::Goal>("/claw")),
      claw_status_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::claw::Status>(
              "/claw")),
      shooter_goal_sender_(
          event_loop->MakeSender<::y2014::control_loops::shooter::Goal>(
              "/shooter")),
      shoot_action_factory_(actors::ShootActor::MakeFactory(event_loop)) {}

void AutonomousActor::PositionClawVertically(double intake_power,
                                             double centering_power) {
  auto builder = claw_goal_sender_.MakeBuilder();
  control_loops::claw::Goal::Builder goal_builder =
      builder.MakeBuilder<control_loops::claw::Goal>();
  goal_builder.add_bottom_angle(0.0);
  goal_builder.add_separation_angle(0.0);
  goal_builder.add_intake(intake_power);
  goal_builder.add_centering(centering_power);

  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawBackIntake() {
  auto builder = claw_goal_sender_.MakeBuilder();
  control_loops::claw::Goal::Builder goal_builder =
      builder.MakeBuilder<control_loops::claw::Goal>();
  goal_builder.add_bottom_angle(-2.273474);
  goal_builder.add_separation_angle(0.0);
  goal_builder.add_intake(12.0);
  goal_builder.add_centering(12.0);
  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawUpClosed() {
  // Move the claw to where we're going to shoot from but keep it closed until
  // it gets there.
  auto builder = claw_goal_sender_.MakeBuilder();
  control_loops::claw::Goal::Builder goal_builder =
      builder.MakeBuilder<control_loops::claw::Goal>();
  goal_builder.add_bottom_angle(0.86);
  goal_builder.add_separation_angle(0.0);
  goal_builder.add_intake(4.0);
  goal_builder.add_centering(1.0);
  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::PositionClawForShot() {
  auto builder = claw_goal_sender_.MakeBuilder();
  control_loops::claw::Goal::Builder goal_builder =
      builder.MakeBuilder<control_loops::claw::Goal>();
  goal_builder.add_bottom_angle(0.86);
  goal_builder.add_separation_angle(0.10);
  goal_builder.add_intake(4.0);
  goal_builder.add_centering(1.0);
  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending claw goal failed\n");
  }
}

void AutonomousActor::SetShotPower(double power) {
  AOS_LOG(INFO, "Setting shot power to %f\n", power);
  auto builder = shooter_goal_sender_.MakeBuilder();
  control_loops::shooter::Goal::Builder goal_builder =
      builder.MakeBuilder<control_loops::shooter::Goal>();
  goal_builder.add_shot_power(power);
  goal_builder.add_shot_requested(false);
  goal_builder.add_unload_requested(false);
  goal_builder.add_load_requested(false);
  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending shooter goal failed\n");
  }
}

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kFastDrive = MakeProfileParameters(3.0, 2.5);
const ProfileParametersT kSlowDrive = MakeProfileParameters(2.5, 2.5);
const ProfileParametersT kFastWithBallDrive = MakeProfileParameters(3.0, 2.0);
const ProfileParametersT kSlowWithBallDrive = MakeProfileParameters(2.5, 2.0);
const ProfileParametersT kFastTurn = MakeProfileParameters(3.0, 10.0);

void AutonomousActor::Shoot() {
  // Shoot.
  aos::common::actions::DoubleParamT param;
  auto shoot_action = shoot_action_factory_.Make(param);
  shoot_action->Start();
  WaitUntilDoneOrCanceled(::std::move(shoot_action));
}

bool AutonomousActor::WaitUntilClawDone() {
  while (true) {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(10),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(10) / 2);
    // Poll the running bit and auto done bits.
    phased_loop.SleepUntilNext();
    claw_status_fetcher_.Fetch();
    claw_goal_fetcher_.Fetch();
    if (ShouldCancel()) {
      return false;
    }
    if (claw_status_fetcher_.get() == nullptr ||
        claw_goal_fetcher_.get() == nullptr) {
      continue;
    }
    bool ans = claw_status_fetcher_->zeroed() &&
               (::std::abs(claw_status_fetcher_->bottom_velocity()) < 1.0) &&
               (::std::abs(claw_status_fetcher_->bottom() -
                           claw_goal_fetcher_->bottom_angle()) < 0.10) &&
               (::std::abs(claw_status_fetcher_->separation() -
                           claw_goal_fetcher_->separation_angle()) < 0.4);
    if (ans) {
      return true;
    }
  }
}

class HotGoalDecoder {
 public:
  HotGoalDecoder(::aos::Fetcher<::y2014::HotGoal> *hot_goal_fetcher)
      : hot_goal_fetcher_(hot_goal_fetcher) {
    ResetCounts();
  }

  void ResetCounts() {
    hot_goal_fetcher_->Fetch();
    if (hot_goal_fetcher_->get()) {
      hot_goal_fetcher_->get()->UnPackTo(&start_counts_);
      start_counts_valid_ = true;
    } else {
      AOS_LOG(WARNING, "no hot goal message. ignoring\n");
      start_counts_valid_ = false;
    }
  }

  void Update() { hot_goal_fetcher_->Fetch(); }

  bool left_triggered() const {
    if (!start_counts_valid_ || !hot_goal_fetcher_->get()) return false;
    return (hot_goal_fetcher_->get()->left_count() - start_counts_.left_count) >
           kThreshold;
  }

  bool right_triggered() const {
    if (!start_counts_valid_ || !hot_goal_fetcher_->get()) return false;
    return (hot_goal_fetcher_->get()->right_count() -
            start_counts_.right_count) > kThreshold;
  }

  bool is_left() const {
    if (!start_counts_valid_ || !hot_goal_fetcher_->get()) return false;
    const uint64_t left_difference =
        hot_goal_fetcher_->get()->left_count() - start_counts_.left_count;
    const uint64_t right_difference =
        hot_goal_fetcher_->get()->right_count() - start_counts_.right_count;
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
    if (!start_counts_valid_ || !hot_goal_fetcher_->get()) return false;
    const uint64_t left_difference =
        hot_goal_fetcher_->get()->left_count() - start_counts_.left_count;
    const uint64_t right_difference =
        hot_goal_fetcher_->get()->right_count() - start_counts_.right_count;
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

  ::y2014::HotGoalT start_counts_;
  bool start_counts_valid_;

  ::aos::Fetcher<::y2014::HotGoal> *hot_goal_fetcher_;
};

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams * /*params*/) {
  enum class AutoVersion : uint8_t {
    kStraight,
    kDoubleHot,
    kSingleHot,
  };

  // The front of the robot is 1.854 meters from the wall
  static const double kShootDistance = 3.15;
  static const double kPickupDistance = 0.5;
  static const double kTurnAngle = 0.3;

  const monotonic_clock::time_point start_time = monotonic_now();
  AOS_LOG(INFO, "Handling auto mode\n");

  AutoVersion auto_version;
  auto_mode_fetcher_.Fetch();
  if (!auto_mode_fetcher_.get()) {
    AOS_LOG(WARNING, "not sure which auto mode to use\n");
    auto_version = AutoVersion::kStraight;
  } else {
    static const double kSelectorMin = 0.2, kSelectorMax = 4.4;

    const double kSelectorStep = (kSelectorMax - kSelectorMin) / 3.0;
    if (auto_mode_fetcher_->voltage() < kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kSingleHot;
    } else if (auto_mode_fetcher_->voltage() < 2 * kSelectorStep + kSelectorMin) {
      auto_version = AutoVersion::kStraight;
    } else {
      auto_version = AutoVersion::kDoubleHot;
    }
  }
  AOS_LOG(INFO, "running auto %" PRIu8 "\n",
          static_cast<uint8_t>(auto_version));

  const ProfileParametersT drive_params =
      (auto_version == AutoVersion::kStraight) ? kFastDrive : kSlowDrive;
  const ProfileParametersT drive_with_ball_params =
      (auto_version == AutoVersion::kStraight) ? kFastWithBallDrive
                                               : kSlowWithBallDrive;

  HotGoalDecoder hot_goal_decoder(&hot_goal_fetcher_);
  // True for left, false for right.
  bool first_shot_left, second_shot_left_default, second_shot_left;

  Reset();

  // Turn the claw on, keep it straight up until the ball has been grabbed.
  AOS_LOG(INFO, "Claw going up at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  PositionClawVertically(12.0, 4.0);
  SetShotPower(115.0);

  // Wait for the ball to enter the claw.
  this_thread::sleep_for(chrono::milliseconds(250));
  if (ShouldCancel()) return true;
  AOS_LOG(INFO, "Readying claw for shot at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  if (ShouldCancel()) return true;
  // Drive to the goal.
  StartDrive(-kShootDistance, 0.0, drive_params, kFastTurn);
  this_thread::sleep_for(chrono::milliseconds(750));
  PositionClawForShot();
  AOS_LOG(INFO, "Waiting until drivetrain is finished\n");
  WaitForDriveProfileDone();
  if (ShouldCancel()) return true;

  hot_goal_decoder.Update();
  if (hot_goal_decoder.is_left()) {
    AOS_LOG(INFO, "first shot left\n");
    first_shot_left = true;
    second_shot_left_default = false;
  } else if (hot_goal_decoder.is_right()) {
    AOS_LOG(INFO, "first shot right\n");
    first_shot_left = false;
    second_shot_left_default = true;
  } else {
    AOS_LOG(INFO, "first shot defaulting left\n");
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
      hot_goal_decoder.Update();
      if (ShouldCancel()) return true;
    } while (!hot_goal_decoder.left_triggered() &&
             (monotonic_now() - start_time) < chrono::seconds(9));
  } else if (auto_version == AutoVersion::kStraight) {
    this_thread::sleep_for(chrono::milliseconds(400));
  }

  // Shoot.
  AOS_LOG(INFO, "Shooting at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  Shoot();
  this_thread::sleep_for(chrono::milliseconds(50));

  if (auto_version == AutoVersion::kDoubleHot) {
    if (ShouldCancel()) return true;
    StartDrive(0, first_shot_left ? -kTurnAngle : kTurnAngle,
               drive_with_ball_params, kFastTurn);
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
  } else if (auto_version == AutoVersion::kSingleHot) {
    AOS_LOG(INFO, "auto done at %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    PositionClawVertically(0.0, 0.0);
    return true;
  }

  {
    if (ShouldCancel()) return true;
    // Intake the new ball.
    AOS_LOG(INFO, "Claw ready for intake at %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    PositionClawBackIntake();
    StartDrive(kShootDistance + kPickupDistance, 0.0, drive_params, kFastTurn);
    AOS_LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
    AOS_LOG(INFO, "Wait for the claw at %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    if (!WaitUntilClawDone()) return true;
  }

  // Drive back.
  {
    AOS_LOG(INFO, "Driving back at %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    StartDrive(-(kShootDistance + kPickupDistance), 0.0, drive_params,
               kFastTurn);
    this_thread::sleep_for(chrono::milliseconds(300));
    hot_goal_decoder.ResetCounts();
    if (ShouldCancel()) return true;
    PositionClawUpClosed();
    if (!WaitUntilClawDone()) return true;
    PositionClawForShot();
    AOS_LOG(INFO, "Waiting until drivetrain is finished\n");
    WaitForDriveProfileDone();
    if (ShouldCancel()) return true;
    if (!WaitUntilClawDone()) return true;
  }

  hot_goal_decoder.Update();
  if (hot_goal_decoder.is_left()) {
    AOS_LOG(INFO, "second shot left\n");
    second_shot_left = true;
  } else if (hot_goal_decoder.is_right()) {
    AOS_LOG(INFO, "second shot right\n");
    second_shot_left = false;
  } else {
    AOS_LOG(INFO, "second shot defaulting %s\n",
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

  AOS_LOG(INFO, "Shooting at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
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
