#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/actions/actions.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/actors/shoot_actor.h"

using ::frc971::control_loops::drivetrain_queue;
using ::frc971::sensors::gyro_reading;

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

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        shot_power_(80.0),
        goal_angle_(0.0),
        separation_angle_(kGrabSeparation),
        velocity_compensation_(0.0),
        intake_power_(0.0),
        was_running_(false) {}

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
    bool last_auto_running = auto_running_;
    auto_running_ = data.GetControlBit(ControlBit::kAutonomous) &&
                    data.GetControlBit(ControlBit::kEnabled);
    if (auto_running_ != last_auto_running) {
      if (auto_running_) {
        StartAuto();
      } else {
        StopAuto();
      }
    }

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    bool is_control_loop_driving = false;
    double left_goal = 0.0;
    double right_goal = 0.0;
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);
    const double kThrottleGain = 1.0 / 2.5;
    if (false && (data.IsPressed(kDriveControlLoopEnable1) ||
                  data.IsPressed(kDriveControlLoopEnable2))) {
      if (data.PosEdge(kDriveControlLoopEnable1) ||
          data.PosEdge(kDriveControlLoopEnable2)) {
        if (drivetrain_queue.position.FetchLatest() &&
            gyro_reading.FetchLatest()) {
          distance_ = (drivetrain_queue.position->left_encoder +
                      drivetrain_queue.position->right_encoder) /
                         2.0 -
                     throttle * kThrottleGain / 2.0;
          angle_ = gyro_reading->angle;
          filtered_goal_distance_ = distance_;
        }
      }
      is_control_loop_driving = true;

      // const double gyro_angle = Gyro.View().angle;
      const double goal_theta = angle_ - wheel * 0.27;
      const double goal_distance = distance_ + throttle * kThrottleGain;
      const double robot_width = 22.0 / 100.0 * 2.54;
      const double kMaxVelocity = 0.6;
      if (goal_distance > kMaxVelocity * 0.02 + filtered_goal_distance_) {
        filtered_goal_distance_ += kMaxVelocity * 0.03;
      } else if (goal_distance <
                 -kMaxVelocity * 0.02 + filtered_goal_distance_) {
        filtered_goal_distance_ -= kMaxVelocity * 0.02;
      } else {
        filtered_goal_distance_ = goal_distance;
      }
      left_goal = filtered_goal_distance_ - robot_width * goal_theta / 2.0;
      right_goal = filtered_goal_distance_ + robot_width * goal_theta / 2.0;
      is_high_gear_ = false;

      LOG(DEBUG, "Left goal %f Right goal %f\n", left_goal, right_goal);
    }
    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .highgear(is_high_gear_)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal)
             .right_goal(right_goal)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = false;
    }
    if (data.PosEdge(kShiftHigh)) {
      is_high_gear_ = true;
    }
  }

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

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
    }
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
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedIntakeGoal);
      } else if (data.IsPressed(kVerticalTuck)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kVerticalTuckGoal);
      } else if (data.IsPressed(kTuck)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedLongShotGoal);
      } else if (data.PosEdge(kCloseShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedMediumShotGoal);
      } else if (data.PosEdge(kFenderShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedShortShotGoal);
      } else if (data.PosEdge(kHumanPlayerShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedHumanShotGoal);
      } else if (data.PosEdge(kUserLeft)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlipped254PassGoal);
      } else if (data.PosEdge(kUserRight)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedDemoShotGoal);
      } else if (data.PosEdge(kTrussShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFlippedTrussShotGoal);
      }
    } else {
      if (data.IsPressed(kIntakeOpenPosition)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kIntakeGoal);
      } else if (data.IsPressed(kVerticalTuck)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kVerticalTuckGoal);
      } else if (data.IsPressed(kTuck)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kLongShotGoal);
      } else if (data.PosEdge(kCloseShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kCloseShotGoal);
      } else if (data.PosEdge(kFenderShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kFenderShotGoal);
      } else if (data.PosEdge(kHumanPlayerShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kHumanShotGoal);
      } else if (data.PosEdge(kUserLeft)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(k254PassGoal);
      } else if (data.PosEdge(kUserRight)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kDemoShotGoal);
      } else if (data.PosEdge(kTrussShot)) {
        action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
        SetGoal(kTrussShotGoal);
      }
    }

    if (data.PosEdge(kFire)) {
      action_queue_.EnqueueAction(actors::MakeShootAction());
    } else if (data.NegEdge(kFire)) {
      action_queue_.CancelCurrentAction();
    }

    action_queue_.Tick();
    if (data.IsPressed(kUnload) || data.IsPressed(kReload)) {
      action_queue_.CancelAllActions();
        LOG(DEBUG, "Canceling\n");
      intake_power_ = 0.0;
      velocity_compensation_ = 0.0;
    }

    // Send out the claw and shooter goals if no actions are running.
    if (!action_queue_.Running()) {
      goal_angle_ += claw_goal_adjust;
      separation_angle_ += claw_separation_adjust;

      // If the action just ended, turn the intake off and stop velocity
      // compensating.
      if (was_running_) {
        intake_power_ = 0.0;
        velocity_compensation_ = 0.0;
      }

      ::frc971::control_loops::drivetrain_queue.status.FetchLatest();
      double goal_angle = goal_angle_;
      if (::frc971::control_loops::drivetrain_queue.status.get()) {
        goal_angle += SpeedToAngleOffset(
            ::frc971::control_loops::drivetrain_queue.status->robot_speed);
      } else {
        LOG_INTERVAL(no_drivetrain_status_);
      }

      if (moving_for_shot_) {
        auto &claw_status = control_loops::claw_queue.status;
        claw_status.FetchLatest();
        if (claw_status.get()) {
          if (::std::abs(claw_status->bottom - goal_angle) < 0.2) {
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
      if (!control_loops::claw_queue.goal.MakeWithBuilder()
               .bottom_angle(goal_angle)
               .separation_angle(separation_angle)
               .intake(intaking ? 12.0
                                : (data.IsPressed(kRollersOut) ? -12.0
                                                               : intake_power_))
               .centering(intaking ? 12.0 : 0.0)
               .Send()) {
        LOG(WARNING, "sending claw goal failed\n");
      }

      if (!control_loops::shooter_queue.goal.MakeWithBuilder()
               .shot_power(shot_power_)
               .shot_requested(data.IsPressed(kFire))
               .unload_requested(data.IsPressed(kUnload))
               .load_requested(data.IsPressed(kReload))
               .Send()) {
        LOG(WARNING, "sending shooter goal failed\n");
      }
    }
    was_running_ = action_queue_.Running();
  }

  double SpeedToAngleOffset(double speed) {
    const ::y2014::constants::Values &values = ::y2014::constants::GetValues();
    // scale speed to a [0.0-1.0] on something close to the max
    // TODO(austin): Change the scale factor for different shots.
    return (speed / values.drivetrain_max_speed) * velocity_compensation_;
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");
    ::frc971::autonomous::autonomous.MakeWithBuilder().run_auto(true).Send();
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    ::frc971::autonomous::autonomous.MakeWithBuilder().run_auto(false).Send();
  }

  bool is_high_gear_;
  double shot_power_;
  double goal_angle_;
  double separation_angle_, shot_separation_angle_;
  double velocity_compensation_;
  // Distance, angle, and filtered goal for closed loop driving.
  double distance_;
  double angle_;
  double filtered_goal_distance_;
  double intake_power_;
  bool was_running_;
  bool moving_for_shot_ = false;

  bool auto_running_ = false;

  ::aos::common::actions::ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2014

int main() {
  ::aos::Init(-1);
  ::y2014::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
