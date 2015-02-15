#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/actions/action_client.h"

using ::frc971::control_loops::drivetrain_queue;
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
  bool Running() { return static_cast<bool>(current_action_); }

 private:
  ::std::unique_ptr<Action> current_action_;
  ::std::unique_ptr<Action> next_action_;
};


class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        was_running_(false) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    bool last_auto_running = auto_running_;
    auto_running_ = data.GetControlBit(ControlBit::kAutonomous) &&
                    data.IsPressed(ControlBit::kEnabled);
    if (auto_running_ != last_auto_running) {
      if (auto_running_) {
        StartAuto();
      } else {
        StopAuto();
      }
    }

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
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
        if (drivetrain_queue.position.FetchLatest() &&
            gyro_reading.FetchLatest()) {
          distance = (drivetrain_queue.position->left_encoder +
                      drivetrain_queue.position->right_encoder) /
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
    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             //.highgear(is_high_gear_)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal)
             .right_goal(right_goal)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
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

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    HandleDrivetrain(data);
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    action_queue_.Tick();
    was_running_ = action_queue_.Running();
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
  bool was_running_;

  bool auto_running_ = false;

  ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
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
