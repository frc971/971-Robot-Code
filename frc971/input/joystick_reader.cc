#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/constants.h"
#include "frc971/queues/other_sensors.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/actions/shoot_action.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/catch_action.q.h"
#include "frc971/actions/shoot_action.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace frc971 {
namespace input {
namespace joysticks {

const ButtonLocation kDriveControlLoopEnable1(1, 7),
                     kDriveControlLoopEnable2(1, 11);
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kCatch(3, 10);

const ButtonLocation kFire(3, 9);
const ButtonLocation kUnload(2, 11);
const ButtonLocation kReload(2, 6);

const ButtonLocation kRollersOut(3, 8);
const ButtonLocation kRollersIn(3, 3);

const ButtonLocation kTuck(3, 4);
const ButtonLocation kIntakePosition(3, 5);
const ButtonLocation kIntakeOpenPosition(3, 11);
//const ButtonLocation kFlipRobot(3, 7);
const JoystickAxis kFlipRobot(3, 3);
// Truss shot. (3, 1)

const ButtonLocation kLongShot(3, 7);
const ButtonLocation kMediumShot(3, 6);
const ButtonLocation kShortShot(3, 2);

struct ClawGoal {
  double angle;
  double separation;
};

struct ShotGoal {
  ClawGoal claw;
  double shot_power;
  bool velocity_compensation;
  double intake_power;
};

const ClawGoal kTuckGoal = {-2.273474, -0.749484};
const ClawGoal kIntakeGoal = {-2.273474, 0.0};
const ClawGoal kIntakeOpenGoal = {-2.0, 1.2};

// TODO(austin): Tune these by hand...
const ClawGoal kFlippedTuckGoal = {2.733474, -0.75};
const ClawGoal kFlippedIntakeGoal = {2.0, 0.0};
const ClawGoal kFlippedIntakeOpenGoal = {0.95, 1.0};

const double kIntakePower = 2.0;
const double kShootSeparation = 0.08;

const ShotGoal kLongShotGoal = {
    {-M_PI / 2.0 + 0.43, kShootSeparation}, 145, false, kIntakePower};
const ShotGoal kMediumShotGoal = {
    {-0.9, kShootSeparation}, 100, true, kIntakePower};
const ShotGoal kShortShotGoal = {
    {-0.04, kShootSeparation}, 10, false, kIntakePower};

const ShotGoal kFlippedLongShotGoal = {
    {M_PI / 2.0 - 0.43, kShootSeparation}, 145, false, kIntakePower};
const ShotGoal kFlippedMediumShotGoal = {
    {0.9, kShootSeparation}, 100, true, kIntakePower};
const ShotGoal kFlippedShortShotGoal = {
    {0.04, kShootSeparation}, 10, false, kIntakePower};

// Makes a new ShootAction action.
::std::unique_ptr<TypedAction< ::frc971::actions::CatchActionGroup>>
MakeCatchAction() {
  return ::std::unique_ptr<TypedAction< ::frc971::actions::CatchActionGroup>>(
      new TypedAction< ::frc971::actions::CatchActionGroup>(
          &::frc971::actions::catch_action));
}

// A queue which queues Actions and cancels them.
class ActionQueue {
 public:
  // Queues up an action for sending.
  void QueueAction(::std::unique_ptr<Action> action) {
    if (current_action_) {
      LOG(INFO, "Queueing action, canceling prior\n");
      current_action_->Cancel();
      next_action_ = ::std::move(action);
    } else {
      LOG(INFO, "Queueing action\n");
      current_action_ = ::std::move(action);
      current_action_->Start();
    }
  }

  // Cancels the current action, and runs the next one when the current one has
  // finished.
  void CancelCurrentAction() {
    LOG(INFO, "Canceling current action\n");
    if (current_action_) {
      current_action_->Cancel();
    }
  }

  // Cancels all running actions.
  void CancelAllActions() {
    LOG(DEBUG, "Cancelling all actions\n");
    if (current_action_) {
      current_action_->Cancel();
    }
    next_action_.reset();
  }

  // Runs the next action when the current one is finished running.
  void Tick() {
    if (current_action_) {
      if (!current_action_->Running()) {
        LOG(INFO, "Action is done.\n");
        current_action_ = ::std::move(next_action_);
        if (current_action_) {
          LOG(INFO, "Running next action\n");
          current_action_->Start();
        }
      }
    }
  }

  // Returns true if any action is running or could be running.
  // For a one cycle faster response, call Tick before running this.
  bool Running() { return (bool)current_action_; }

 private:
  ::std::unique_ptr<Action> current_action_;
  ::std::unique_ptr<Action> next_action_;
};


class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        shot_power_(80.0),
        goal_angle_(0.0),
        separation_angle_(0.0),
        velocity_compensation_(false),
        intake_power_(0.0),
        was_running_(false) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    if (data.GetControlBit(ControlBit::kAutonomous)) {
      if (data.PosEdge(ControlBit::kEnabled)){
        LOG(INFO, "Starting auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(true)
            .Send();
      } else if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "Stopping auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(false)
            .Send();
      }
    } else {
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
      // TODO(austin): Static sucks!
      static double distance = 0.0;
      static double angle = 0.0;
      static double filtered_goal_distance = 0.0;
      if (data.PosEdge(kDriveControlLoopEnable1) ||
          data.PosEdge(kDriveControlLoopEnable2)) {
        if (drivetrain.position.FetchLatest() && gyro_reading.FetchLatest()) {
          distance = (drivetrain.position->left_encoder +
                      drivetrain.position->right_encoder) /
                         2.0 -
                     throttle * kThrottleGain / 2.0;
          angle = gyro_reading->angle;
          filtered_goal_distance = distance;
        }
      }
      is_control_loop_driving = true;

      // const double gyro_angle = Gyro.View().angle;
      const double goal_theta = angle - wheel * 0.27;
      const double goal_distance = distance + throttle * kThrottleGain;
      const double robot_width = 22.0 / 100.0 * 2.54;
      const double kMaxVelocity = 0.6;
      if (goal_distance > kMaxVelocity * 0.02 + filtered_goal_distance) {
        filtered_goal_distance += kMaxVelocity * 0.02;
      } else if (goal_distance <
                 -kMaxVelocity * 0.02 + filtered_goal_distance) {
        filtered_goal_distance -= kMaxVelocity * 0.02;
      } else {
        filtered_goal_distance = goal_distance;
      }
      left_goal = filtered_goal_distance - robot_width * goal_theta / 2.0;
      right_goal = filtered_goal_distance + robot_width * goal_theta / 2.0;
      is_high_gear_ = false;

      LOG(DEBUG, "Left goal %f Right goal %f\n", left_goal, right_goal);
    }
    if (!drivetrain.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .highgear(is_high_gear_)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal)
             .right_goal(right_goal)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
    if (data.PosEdge(kShiftHigh)) {
      is_high_gear_ = false;
    }
    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = true;
    }
  }

  void SetGoal(ClawGoal goal) {
    goal_angle_ = goal.angle;
    separation_angle_ = goal.separation;
    velocity_compensation_ = false;
    intake_power_ = 0.0;
  }

  void SetGoal(ShotGoal goal) {
    goal_angle_ = goal.claw.angle;
    separation_angle_ = goal.claw.separation;
    shot_power_ = goal.shot_power;
    velocity_compensation_ = goal.velocity_compensation;
    intake_power_ = goal.intake_power;
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    HandleDrivetrain(data);
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
    }
    if (data.IsPressed(kRollersIn) || data.IsPressed(kRollersOut)) {
      intake_power_ = 0.0;
    }

    if (data.GetAxis(kFlipRobot) < 0.5) {
      if (data.IsPressed(kIntakeOpenPosition)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedIntakeGoal);
      } else if (data.IsPressed(kTuck)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedLongShotGoal);
      } else if (data.PosEdge(kMediumShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedMediumShotGoal);
      } else if (data.PosEdge(kShortShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kFlippedShortShotGoal);
      }
    } else {
      if (data.IsPressed(kIntakeOpenPosition)) {
        action_queue_.CancelAllActions();
        SetGoal(kIntakeOpenGoal);
      } else if (data.IsPressed(kIntakePosition)) {
        action_queue_.CancelAllActions();
        SetGoal(kIntakeGoal);
      } else if (data.IsPressed(kTuck)) {
        action_queue_.CancelAllActions();
        SetGoal(kTuckGoal);
      } else if (data.PosEdge(kLongShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kLongShotGoal);
      } else if (data.PosEdge(kMediumShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kMediumShotGoal);
      } else if (data.PosEdge(kShortShot)) {
        action_queue_.CancelAllActions();
        SetGoal(kShortShotGoal);
      }
    }

    if (data.PosEdge(kCatch)) {
      auto catch_action = MakeCatchAction();
      catch_action->GetGoal()->catch_angle = goal_angle_;
      action_queue_.QueueAction(::std::move(catch_action));
    }

    if (data.PosEdge(kFire)) {
      action_queue_.QueueAction(actions::MakeShootAction());
    }

    action_queue_.Tick();
    if (data.IsPressed(kUnload) || data.IsPressed(kReload)) {
      action_queue_.CancelAllActions();
      intake_power_ = 0.0;
      velocity_compensation_ = false;
    }

    // Send out the claw and shooter goals if no actions are running.
    if (!action_queue_.Running()) {
      // If the action just ended, turn the intake off and stop velocity
      // compensating.
      if (was_running_) {
        intake_power_ = 0.0;
        velocity_compensation_ = false;
      }

      control_loops::drivetrain.status.FetchLatest();
      const double goal_angle =
          goal_angle_ +
          (velocity_compensation_
               ? SpeedToAngleOffset(
                     control_loops::drivetrain.status->robot_speed)
               : 0.0);

      bool intaking = data.IsPressed(kRollersIn) ||
                      data.IsPressed(kIntakePosition) ||
                      data.IsPressed(kIntakeOpenPosition);
      if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
               .bottom_angle(goal_angle)
               .separation_angle(separation_angle_)
               .intake(intaking ? 12.0
                                : (data.IsPressed(kRollersOut) ? -12.0
                                                               : intake_power_))
               .centering(intaking ? 12.0 : 0.0)
               .Send()) {
        LOG(WARNING, "sending claw goal failed\n");
      }

      if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
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
    const frc971::constants::Values &values = frc971::constants::GetValues();
    // scale speed to a [0.0-1.0] on something close to the max
    // TODO(austin): Change the scale factor for different shots.
    return (speed / values.drivetrain_max_speed) * 0.3;
  }

 private:
  bool is_high_gear_;
  double shot_power_;
  double goal_angle_;
  double separation_angle_;
  bool velocity_compensation_;
  double intake_power_;
  bool was_running_;
  
  ActionQueue action_queue_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
