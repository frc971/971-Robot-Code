#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/othersensors.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/actions/shoot_action.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::othersensors;

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

const ButtonLocation kFire(3, 11);
const ButtonLocation kUnload(2, 11);
const ButtonLocation kReload(2, 6);

const ButtonLocation kRollersOut(3, 12);
const ButtonLocation kRollersIn(3, 10);

const ButtonLocation kTuck(3, 8);
const ButtonLocation kIntakeOpenPosition(3, 9);
const ButtonLocation kIntakePosition(3, 7);

const ButtonLocation kLongShot(3, 5);
const ButtonLocation kMediumShot(3, 3);
const ButtonLocation kShortShot(3, 6);

struct ClawGoal {
  double angle;
  double separation;
};

const ClawGoal kTuckGoal = {-2.273474, -0.749484};
const ClawGoal kIntakeGoal = {-2.273474, 0.0};
const ClawGoal kIntakeOpenGoal = {-2.0, 1.2};

const ClawGoal kLongShotGoal = {-M_PI / 2.0 + 0.43, 0.10};
const ClawGoal kMediumShotGoal = {-0.9, 0.10};
const ClawGoal kShortShotGoal = {-0.04, 0.11};

class Action {
 public:
  // Cancels the action.
  void Cancel() { DoCancel(); }
  // Returns true if the action is currently running.
  bool Running() { return DoRunning(); }
  // Starts the action.
  void Start() { DoStart(); }

 private:
  virtual void DoCancel() = 0;
  virtual bool DoRunning() = 0;
  virtual void DoStart() = 0;
};

// Templated subclass to hold the type information.
template <typename T>
class TypedAction : public Action {
 public:
  typedef typename std::remove_reference<
      decltype(*(static_cast<T *>(NULL)->goal.MakeMessage().get()))>::type
        GoalType;

  TypedAction(T *queue_group)
      : queue_group_(queue_group),
        goal_(queue_group_->goal.MakeMessage()),
        has_started_(false) {}

  // Returns the current goal that will be sent when the action is sent.
  GoalType *GetGoal() { return goal_.get(); }

  ~TypedAction() {
    LOG(INFO, "Calling destructor\n");
    DoCancel();
  }

 private:
  // Cancels the action.
  virtual void DoCancel() {
    LOG(INFO, "Canceling action\n");
    queue_group_->goal.MakeWithBuilder().run(false).Send();
  }

  // Returns true if the action is running or we don't have an initial response
  // back from it to signal whether or not it is running.
  virtual bool DoRunning() {
    if (has_started_) {
      queue_group_->status.FetchLatest();
    } else if (queue_group_->status.FetchLatest()) {
      if (queue_group_->status->running) {
        // Wait until it reports that it is running to start.
        has_started_ = true;
      }
    }
    return !has_started_ ||
           (queue_group_->status.get() && queue_group_->status->running);
  }

  // Starts the action if a goal has been created.
  virtual void DoStart() {
    if (goal_) {
      goal_->run = true;
      goal_.Send();
      has_started_ = false;
      LOG(INFO, "Starting action\n");
    } else {
      has_started_ = true;
    }
  }

  T *queue_group_;
  ::aos::ScopedMessagePtr<GoalType> goal_;
  // Track if we have seen a response to the start message.
  // If we haven't, we are considered running regardless.
  bool has_started_;
};

// Makes a new ShootAction action.
::std::unique_ptr<TypedAction< ::frc971::actions::ShootActionQueueGroup>>
MakeShootAction() {
  return ::std::unique_ptr<
      TypedAction< ::frc971::actions::ShootActionQueueGroup>>(
      new TypedAction< ::frc971::actions::ShootActionQueueGroup>(
          &::frc971::actions::shoot_action));
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
    LOG(INFO, "Canceling all actions\n");
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
        separation_angle_(0.0) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    if (data.GetControlBit(ControlBit::kAutonomous)) {
      if (data.PosEdge(ControlBit::kEnabled)){
        LOG(INFO, "Starting auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(true).Send();
      } else if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "Stopping auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(false).Send();
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
        if (drivetrain.position.FetchLatest() && othersensors.FetchLatest()) {
          distance = (drivetrain.position->left_encoder +
                      drivetrain.position->right_encoder) /
                         2.0 -
                     throttle * kThrottleGain / 2.0;
          angle = othersensors->gyro_angle;
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
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    HandleDrivetrain(data);
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
    }

    if (data.IsPressed(kIntakeOpenPosition)) {
      action_queue_.CancelAllActions();
      SetGoal(kIntakeOpenGoal);
    } else if (data.IsPressed(kIntakePosition)) {
      action_queue_.CancelAllActions();
      SetGoal(kIntakeGoal);
    } else if (data.IsPressed(kTuck)) {
      action_queue_.CancelAllActions();
      SetGoal(kTuckGoal);
    }

    if (data.PosEdge(kLongShot)) {
      auto shoot_action = MakeShootAction();

      shot_power_ = 160.0;
      shoot_action->GetGoal()->shot_power = shot_power_;
      shoot_action->GetGoal()->shot_angle = kLongShotGoal.angle;
      SetGoal(kLongShotGoal);

      action_queue_.QueueAction(::std::move(shoot_action));
    } else if (data.PosEdge(kMediumShot)) {
      auto shoot_action = MakeShootAction();

      shot_power_ = 100.0;
      shoot_action->GetGoal()->shot_power = shot_power_;
      shoot_action->GetGoal()->shot_angle = kMediumShotGoal.angle;
      SetGoal(kMediumShotGoal);

      action_queue_.QueueAction(::std::move(shoot_action));
    } else if (data.PosEdge(kShortShot)) {
      auto shoot_action = MakeShootAction();

      shot_power_ = 20.0;
      shoot_action->GetGoal()->shot_power = shot_power_;
      shoot_action->GetGoal()->shot_angle = kShortShotGoal.angle;
      SetGoal(kShortShotGoal);

      action_queue_.QueueAction(::std::move(shoot_action));
    }

    action_queue_.Tick();
    if (data.IsPressed(kUnload) || data.IsPressed(kReload)) {
      action_queue_.CancelAllActions();
    }

    // Send out the claw and shooter goals if no actions are running.
    if (!action_queue_.Running()) {
      if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
               .bottom_angle(goal_angle_)
               .separation_angle(separation_angle_)
               .intake(data.IsPressed(kRollersIn)
                           ? 12.0
                           : (data.IsPressed(kRollersOut) ? -12.0 : 0.0))
               .centering(data.IsPressed(kRollersIn) ? 12.0 : 0.0)
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
  }

 private:
  bool is_high_gear_;
  double shot_power_;
  double goal_angle_;
  double separation_angle_;
  
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
