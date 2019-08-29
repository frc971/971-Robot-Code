#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"

#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2014/actors/shoot_actor.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw_goal_generated.h"
#include "y2014/control_loops/claw/claw_status_generated.h"
#include "y2014/control_loops/drivetrain/drivetrain_base.h"
#include "y2014/control_loops/shooter/shooter_goal_generated.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

#define OLD_DS 0

namespace y2014 {
namespace input {
namespace joysticks {

const ButtonLocation kDriveControlLoopEnable1(1, 7),
                     kDriveControlLoopEnable2(1, 11);
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 3), kShiftLow(2, 1);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kCatch(3, 10);

#if OLD_DS
const ButtonLocation kFire(3, 11);
const ButtonLocation kUnload(1, 4);
const ButtonLocation kReload(1, 2);

const ButtonLocation kRollersOut(3, 12);
const ButtonLocation kRollersIn(3, 7);

const ButtonLocation kTuck(3, 9);
const ButtonLocation kIntakePosition(3, 8);
const ButtonLocation kIntakeOpenPosition(3, 10);
const ButtonLocation kVerticalTuck(3, 1);
const JoystickAxis kFlipRobot(3, 3);

const ButtonLocation kLongShot(3, 5);
const ButtonLocation kCloseShot(3, 2);
const ButtonLocation kFenderShot(3, 3);
const ButtonLocation kTrussShot(2, 11);
const ButtonLocation kHumanPlayerShot(3, 2);
#else
const ButtonLocation kFire(3, 9);
const ButtonLocation kUnload(1, 4);
const ButtonLocation kReload(1, 2);

const ButtonLocation kRollersOut(3, 8);
const ButtonLocation kRollersIn(3, 3);

const ButtonLocation kTuck(3, 4);
const ButtonLocation kIntakePosition(3, 5);
const ButtonLocation kIntakeOpenPosition(3, 11);
const ButtonLocation kVerticalTuck(2, 6);
const JoystickAxis kFlipRobot(3, 3);

const ButtonLocation kLongShot(3, 7);
const ButtonLocation kCloseShot(3, 6);
const ButtonLocation kFenderShot(3, 2);
const ButtonLocation kTrussShot(2, 11);
const ButtonLocation kHumanPlayerShot(3, 1);
#endif

const ButtonLocation kUserLeft(2, 7);
const ButtonLocation kUserRight(2, 10);

const JoystickAxis kAdjustClawGoal(3, 2);
const JoystickAxis kAdjustClawSeparation(3, 1);

struct ClawGoal {
  double angle;
  double separation;
};

struct ShotGoal {
  ClawGoal claw;
  double shot_power;
  double velocity_compensation;
  double intake_power;
};

const double kIntakePower = 4.0;
// In case we have to quickly adjust it.
const double kGrabSeparation = 0;
const double kShootSeparation = 0.11 + kGrabSeparation;

const ClawGoal kTuckGoal = {-2.273474, -0.749484};
const ClawGoal kVerticalTuckGoal = {0, kGrabSeparation};
const ClawGoal kIntakeGoal = {-2.24, kGrabSeparation};
const ClawGoal kIntakeOpenGoal = {-2.0, 1.1};

// TODO(austin): Tune these by hand...
const ClawGoal kFlippedTuckGoal = {2.733474, -0.75};
const ClawGoal kFlippedIntakeGoal = {2.0, kGrabSeparation};
const ClawGoal kFlippedIntakeOpenGoal = {0.95, 1.0};

// 34" between near edge of colored line and rear edge of bumper.
// Only works running?
const ShotGoal kLongShotGoal = {
    {-1.08, kShootSeparation}, 145, 0.04, kIntakePower};
// old 34" {-1.06, kShootSeparation}, 140, 0.04, kIntakePower};
const ShotGoal kFlippedLongShotGoal = {
    {0.96, kShootSeparation}, 145, 0.09, kIntakePower};
// old 34" {0.96, kShootSeparation}, 140, 0.09, kIntakePower};

// 78" between near edge of colored line and rear edge of bumper.
const ShotGoal kCloseShotGoal = {
    {-0.95, kShootSeparation}, 105, 0.2, kIntakePower};
// 3/4" plunger {-0.90, kShootSeparation}, 105, 0.2, kIntakePower};
const ShotGoal kFlippedMediumShotGoal = {
    {0.865, kShootSeparation}, 120, 0.2, kIntakePower};
// 3/4" plunger {0.80, kShootSeparation}, 105, 0.2, kIntakePower};

// Shot from the fender.
const ShotGoal kFenderShotGoal = {
    {-0.68, kShootSeparation}, 115.0, 0.0, kIntakePower};
const ShotGoal kFlippedShortShotGoal = {
    {0.63, kShootSeparation}, 115.0, 0.0, kIntakePower};

const ShotGoal kHumanShotGoal = {
    {-0.90, kShootSeparation}, 140, 0.04, kIntakePower};
const ShotGoal kFlippedHumanShotGoal = {
    {0.90, kShootSeparation}, 140, 0, kIntakePower};
const ShotGoal kTrussShotGoal = {
    {-0.68, kShootSeparation}, 88.0, 0.4, kIntakePower};
const ShotGoal kFlippedTrussShotGoal = {
    {0.68, kShootSeparation}, 92.0, 0.4, kIntakePower};

const ShotGoal kFlippedDemoShotGoal = {
    {1.0, kShootSeparation}, 65.0, 0.0, kIntakePower};
const ShotGoal kDemoShotGoal = {
    {-1.0, kShootSeparation}, 50.0, 0.0, kIntakePower};

const ClawGoal k254PassGoal = {-1.95, kGrabSeparation};
const ClawGoal kFlipped254PassGoal = {1.96, kGrabSeparation};

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop, control_loops::GetDrivetrainConfig(),
            ::aos::input::DrivetrainInputReader::InputType::kSteeringWheel, {}),
        claw_status_fetcher_(
            event_loop->MakeFetcher<::y2014::control_loops::claw::Status>(
                "/claw")),
        claw_goal_sender_(
            event_loop->MakeSender<::y2014::control_loops::claw::Goal>(
                "/claw")),
        shooter_goal_sender_(
            event_loop->MakeSender<::y2014::control_loops::shooter::Goal>(
                "/shooter")),
        drivetrain_status_fetcher_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
                    "/drivetrain")),
        shot_power_(80.0),
        goal_angle_(0.0),
        separation_angle_(kGrabSeparation),
        velocity_compensation_(0.0),
        intake_power_(0.0),
        shoot_action_factory_(actors::ShootActor::MakeFactory(event_loop)) {}

  void SetGoal(ClawGoal goal) {
    goal_angle_ = goal.angle;
    separation_angle_ = goal.separation;
    moving_for_shot_ = false;
    velocity_compensation_ = 0.0;
    intake_power_ = 0.0;
  }

  void SetGoal(ShotGoal goal) {
    goal_angle_ = goal.claw.angle;
    shot_separation_angle_ = goal.claw.separation;
    separation_angle_ = kGrabSeparation;
    moving_for_shot_ = true;
    shot_power_ = goal.shot_power;
    velocity_compensation_ = goal.velocity_compensation;
    intake_power_ = goal.intake_power;
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) override {
    if (data.IsPressed(kRollersIn) || data.IsPressed(kRollersOut)) {
      intake_power_ = 0.0;
      separation_angle_ = kGrabSeparation;
      moving_for_shot_ = false;
    }

    static const double kAdjustClawGoalDeadband = 0.08;
    double claw_goal_adjust = data.GetAxis(kAdjustClawGoal);
    if (OLD_DS || ::std::abs(claw_goal_adjust) < kAdjustClawGoalDeadband) {
      claw_goal_adjust = 0;
    } else {
      claw_goal_adjust = (claw_goal_adjust -
                          ((claw_goal_adjust < 0) ? -kAdjustClawGoalDeadband
                                                  : kAdjustClawGoalDeadband)) *
                         0.035;
    }
    double claw_separation_adjust = data.GetAxis(kAdjustClawSeparation);
    if (OLD_DS ||
        ::std::abs(claw_separation_adjust) < kAdjustClawGoalDeadband) {
      claw_separation_adjust = 0;
    } else {
      claw_separation_adjust =
          (claw_separation_adjust -
           ((claw_separation_adjust < 0) ? -kAdjustClawGoalDeadband
                                         : kAdjustClawGoalDeadband)) *
          -0.035;
    }

#if OLD_DS
    if (data.IsPressed(kFenderShot)) {
#else
    if (data.GetAxis(kFlipRobot) > 0.9) {
#endif
      claw_goal_adjust += claw_separation_adjust;
      claw_goal_adjust *= -1;

      if (data.IsPressed(kIntakeOpenPosition)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedIntakeGoal);
      } else if (data.IsPressed(kVerticalTuck)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kVerticalTuckGoal);
      } else if (data.IsPressed(kTuck)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedLongShotGoal);
      } else if (data.PosEdge(kCloseShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedMediumShotGoal);
      } else if (data.PosEdge(kFenderShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedShortShotGoal);
      } else if (data.PosEdge(kHumanPlayerShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedHumanShotGoal);
      } else if (data.PosEdge(kUserLeft)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlipped254PassGoal);
      } else if (data.PosEdge(kUserRight)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedDemoShotGoal);
      } else if (data.PosEdge(kTrussShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedTrussShotGoal);
      }
    } else {
      if (data.IsPressed(kIntakeOpenPosition)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kIntakeGoal);
      } else if (data.IsPressed(kVerticalTuck)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kVerticalTuckGoal);
      } else if (data.IsPressed(kTuck)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kLongShotGoal);
      } else if (data.PosEdge(kCloseShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kCloseShotGoal);
      } else if (data.PosEdge(kFenderShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kFenderShotGoal);
      } else if (data.PosEdge(kHumanPlayerShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kHumanShotGoal);
      } else if (data.PosEdge(kUserLeft)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(k254PassGoal);
      } else if (data.PosEdge(kUserRight)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kDemoShotGoal);
      } else if (data.PosEdge(kTrussShot)) {
        CancelAllActions();
        AOS_LOG(DEBUG, "Canceling\n");
        SetGoal(kTrussShotGoal);
      }
    }

    if (data.PosEdge(kFire)) {
      aos::common::actions::DoubleParamT param;
      EnqueueAction(shoot_action_factory_.Make(param));
    } else if (data.NegEdge(kFire)) {
      CancelCurrentAction();
    }

    if (data.IsPressed(kUnload) || data.IsPressed(kReload)) {
      CancelAllActions();
      AOS_LOG(DEBUG, "Canceling\n");
      intake_power_ = 0.0;
      velocity_compensation_ = 0.0;
    }

    // Send out the claw and shooter goals if no actions are running.
    if (!ActionRunning()) {
      goal_angle_ += claw_goal_adjust;
      separation_angle_ += claw_separation_adjust;

      // If the action just ended, turn the intake off and stop velocity
      // compensating.
      if (was_running_action()) {
        intake_power_ = 0.0;
        velocity_compensation_ = 0.0;
      }

      drivetrain_status_fetcher_.Fetch();
      double goal_angle = goal_angle_;
      if (drivetrain_status_fetcher_.get()) {
        goal_angle +=
            SpeedToAngleOffset(drivetrain_status_fetcher_->robot_speed());
      } else {
        AOS_LOG_INTERVAL(no_drivetrain_status_);
      }

      if (moving_for_shot_) {
        claw_status_fetcher_.Fetch();
        if (claw_status_fetcher_.get()) {
          if (::std::abs(claw_status_fetcher_->bottom() - goal_angle) < 0.2) {
            moving_for_shot_ = false;
            separation_angle_ = shot_separation_angle_;
          }
        }
      }

      double separation_angle = separation_angle_;

      if (data.IsPressed(kCatch)) {
        const double kCatchSeparation = 1.0;
        goal_angle -= kCatchSeparation / 2.0;
        separation_angle = kCatchSeparation;
      }

      bool intaking =
          data.IsPressed(kRollersIn) || data.IsPressed(kIntakePosition) ||
          data.IsPressed(kIntakeOpenPosition) || data.IsPressed(kCatch);
      {
        auto builder = claw_goal_sender_.MakeBuilder();
        control_loops::claw::Goal::Builder goal_builder =
            builder.MakeBuilder<control_loops::claw::Goal>();
        goal_builder.add_bottom_angle(goal_angle);
        goal_builder.add_separation_angle(separation_angle);
        goal_builder.add_intake(
            intaking ? 12.0
                     : (data.IsPressed(kRollersOut) ? -12.0 : intake_power_));
        goal_builder.add_centering(intaking ? 12.0 : 0.0);

        if (!builder.Send(goal_builder.Finish())) {
          AOS_LOG(WARNING, "sending claw goal failed\n");
        }
      }

      {
        auto builder = shooter_goal_sender_.MakeBuilder();
        control_loops::shooter::Goal::Builder goal_builder =
            builder.MakeBuilder<control_loops::shooter::Goal>();
        goal_builder.add_shot_power(shot_power_);
        goal_builder.add_shot_requested(data.IsPressed(kFire));
        goal_builder.add_unload_requested(data.IsPressed(kUnload));
        goal_builder.add_load_requested(data.IsPressed(kReload));
        if (!builder.Send(goal_builder.Finish())) {
          AOS_LOG(WARNING, "sending shooter goal failed\n");
        }
      }
    }
  }

  double SpeedToAngleOffset(double speed) {
    const ::y2014::constants::Values &values = ::y2014::constants::GetValues();
    // scale speed to a [0.0-1.0] on something close to the max
    // TODO(austin): Change the scale factor for different shots.
    return (speed / values.drivetrain_max_speed) * velocity_compensation_;
  }

 private:
  ::aos::Fetcher<::y2014::control_loops::claw::Status> claw_status_fetcher_;
  ::aos::Sender<::y2014::control_loops::claw::Goal> claw_goal_sender_;
  ::aos::Sender<::y2014::control_loops::shooter::Goal> shooter_goal_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  double shot_power_;
  double goal_angle_;
  double separation_angle_, shot_separation_angle_;
  double velocity_compensation_;
  double intake_power_;
  bool moving_for_shot_ = false;

  actors::ShootActor::Factory shoot_action_factory_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::std::chrono::milliseconds(200), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2014

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2014::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
