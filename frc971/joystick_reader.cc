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
#include "aos/common/actions/actions.h"

#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"

using ::frc971::control_loops::claw_queue;
using ::frc971::control_loops::drivetrain_queue;
using ::frc971::control_loops::fridge_queue;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace frc971 {
namespace input {
namespace joysticks {

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

// TODO(danielp): Real buttons for all of these.
const ButtonLocation kElevatorUp(99, 99);
const ButtonLocation kElevatorDown(99, 99);
const ButtonLocation kArmUp(99, 99);
const ButtonLocation kArmDown(99, 99);
const ButtonLocation kClawUp(99, 99);
const ButtonLocation kClawDown(99, 99);

// Testing mode.
const double kElevatorVelocity = 0.1;
const double kArmVelocity = 0.2;
const double kClawVelocity = 0.2;
// TODO(danielp): Verify.
const double kJoystickDt = 0.01;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : was_running_(false) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
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
      if (data.GetControlBit(ControlBit::kTestMode)) {
        HandleTest(data);
      } else {
        HandleTeleop(data);
      }
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);

    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(false)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

  void HandleTest(const ::aos::input::driver_station::Data &data) {
    if (action_queue_.Running()) {
      // We don't really want any actions running.
      LOG(DEBUG, "Cancelling actions for test mode.\n");
      action_queue_.CancelAllActions();
    }

    if (data.GetControlBit(ControlBit::kEnabled)) {
      if (data.PosEdge(ControlBit::kEnabled)) {
        // If we got enabled, wait for everything to zero.
        LOG(INFO, "Waiting for zero.\n");
        waiting_for_zero_ = true;
      }
      if (waiting_for_zero_) {
        claw_queue.status.FetchLatest();
        fridge_queue.status.FetchLatest();
        if (!claw_queue.status.get()) {
          LOG(ERROR, "Got no claw status packet.\n");
          // Not safe to continue.
          return;
        }
        if (!fridge_queue.status.get()) {
          LOG(ERROR, "Got no fridge status packet.\n");
          return;
        }

        if (claw_queue.status->zeroed && fridge_queue.status->zeroed) {
          LOG(INFO, "Zeroed! Starting test mode.\n");
          waiting_for_zero_ = false;

          // Set the initial goals to where we are now.
          elevator_goal_ = fridge_queue.status->height;
          arm_goal_ = fridge_queue.status->angle;
          claw_goal_ = claw_queue.status->angle;
        } else {
          return;
        }
      }

      // These buttons move a subsystem up or down for as long as they are
      // pressed, at low velocity.
      if (data.IsPressed(kElevatorUp)) {
        elevator_goal_ += kElevatorVelocity * kJoystickDt;
      }
      if (data.IsPressed(kElevatorDown)) {
        elevator_goal_ -= kElevatorVelocity * kJoystickDt;
      }
      if (data.IsPressed(kArmUp)) {
        arm_goal_ += kArmVelocity * kJoystickDt;
      }
      if (data.IsPressed(kArmDown)) {
        arm_goal_ -= kArmVelocity * kJoystickDt;
      }
      if (data.IsPressed(kClawUp)) {
        claw_goal_ += kClawVelocity * kJoystickDt;
      }
      if (data.IsPressed(kClawDown)) {
        claw_goal_ -= kClawVelocity * kJoystickDt;
      }

      if (!fridge_queue.goal.MakeWithBuilder()
          .height(elevator_goal_)
          .angle(arm_goal_)
          .Send()) {
        LOG(ERROR, "Sending fridge goal failed.\n");
      }
      if (!claw_queue.goal.MakeWithBuilder()
          .angle(claw_goal_)
          .Send()) {
        LOG(ERROR, "Sending claw goal failed.\n");
      }
    }
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

  bool was_running_;

  // Previous goals for systems.
  double elevator_goal_ = 0.0;
  double arm_goal_ = 0.0;
  double claw_goal_ = 0.0;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = false;

  bool auto_running_ = false;

  ::aos::common::actions::ActionQueue action_queue_;

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
