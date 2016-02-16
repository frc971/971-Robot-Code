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
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

#include "y2016/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2016::control_loops::shooter::shooter_queue;
using ::y2016::control_loops::superstructure_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace y2016 {
namespace input {
namespace joysticks {

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

// Buttons on the lexan driver station to get things running on bring-up day.
const ButtonLocation kTest1(3, 6);
const ButtonLocation kTest2(3, 2);
const ButtonLocation kTest3(3, 11);
const ButtonLocation kTest4(3, 9);
const ButtonLocation kTest5(3, 8);
const ButtonLocation kTest6(3, 3);
const ButtonLocation kTest7(3, 5);
const ButtonLocation kTest8(3, 4);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        intake_goal_(0.0),
        shoulder_goal_(M_PI / 2.0),
        wrist_goal_(0.0) {}

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

    if (data.PosEdge(kShiftHigh)) {
      is_high_gear_ = false;
    }

    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = true;
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    if (data.PosEdge(ControlBit::kEnabled)) {
      // If we got enabled, wait for everything to zero.
      LOG(INFO, "Waiting for zero.\n");
      waiting_for_zero_ = true;
    }

    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
    }

    if (superstructure_queue.status.get() &&
        superstructure_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(INFO, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;
      }
    } else {
      waiting_for_zero_ = true;
    }

    // TODO(robot bringup): Populate these with test goals.
    if (data.PosEdge(kTest1)) {
    }

    if (data.PosEdge(kTest2)) {
    }

    if (data.PosEdge(kTest3)) {
    }

    if (data.PosEdge(kTest4)) {
    }

    if (data.PosEdge(kTest5)) {
    }

    if (data.PosEdge(kTest6)) {
    }

    if (data.PosEdge(kTest7)) {
    }

    if (data.PosEdge(kTest8)) {
    }

    if (!waiting_for_zero_) {
      if (!action_queue_.Running()) {
        auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
        new_superstructure_goal->angle_intake = intake_goal_;
        new_superstructure_goal->angle_shoulder = shoulder_goal_;
        new_superstructure_goal->angle_wrist = wrist_goal_;
        new_superstructure_goal->max_angular_velocity_intake = 0.1;
        new_superstructure_goal->max_angular_velocity_shoulder = 0.1;
        new_superstructure_goal->max_angular_velocity_wrist = 0.1;
        new_superstructure_goal->max_angular_acceleration_intake = 0.05;
        new_superstructure_goal->max_angular_acceleration_shoulder = 0.05;
        new_superstructure_goal->max_angular_acceleration_wrist = 0.05;
        new_superstructure_goal->voltage_rollers = 0.0;

        if (!new_superstructure_goal.Send()) {
          LOG(ERROR, "Sending superstructure goal failed.\n");
        } else {
          LOG(DEBUG, "sending goals: intake: %f, shoulder: %f, wrist: %f\n",
              intake_goal_, shoulder_goal_, wrist_goal_);
        }

        if (!shooter_queue.goal.MakeWithBuilder()
                 .angular_velocity(0.0)
                 .clamp_open(false)
                 .push_to_shooter(false)
                 .Send()) {
          LOG(ERROR, "Sending shooter goal failed.\n");
        }
      }
    }

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
  // Whatever these are set to are our default goals to send out after zeroing.
  double intake_goal_;
  double shoulder_goal_;
  double wrist_goal_;

  bool was_running_ = false;
  bool auto_running_ = false;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  ::aos::common::actions::ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2016

int main() {
  ::aos::Init(-1);
  ::y2016::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
