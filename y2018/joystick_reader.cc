#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/common/actions/actions.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/linux_code/init.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2018::control_loops::superstructure_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

namespace y2018 {
namespace input {
namespace joysticks {

//TODO(Neil): Change button locations to real ones on driverstation.
const ButtonLocation kIntakeDown(3, 9);
const POVLocation kIntakeUp(3, 90);
const ButtonLocation kIntakeIn(3, 12);
const ButtonLocation kIntakeOut(3, 8);

const ButtonLocation kArmDown(3, 3);
const ButtonLocation kArmSwitch(3, 7);
const ButtonLocation kArmScale(3, 6);

const ButtonLocation kClawOpen(3, 5);
const ButtonLocation kClawClose(3, 4);

const ButtonLocation kForkDeploy(3, 11);
const ButtonLocation kForkStow(3, 10);

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kSteeringWheel,
        ::y2018::control_loops::drivetrain::GetDrivetrainConfig());
  }

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

    if (!auto_running_) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }

    // Process any pending actions.
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
    robot_velocity_ = drivetrain_input_reader_->robot_velocity();
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    if (data.IsPressed(kIntakeUp)) {
      // Bring in the intake.
      intake_goal_ = -M_PI; //TODO(Neil): Add real value here once we have one.
    }
    if (data.IsPressed(kIntakeDown)) {
      // Deploy the intake.
      intake_goal_ = 0; //TODO(Neil): Add real value here once we have one.
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    new_superstructure_goal->intake.left_intake_angle = intake_goal_;
    new_superstructure_goal->intake.right_intake_angle = intake_goal_;

    if (data.IsPressed(kIntakeIn)) {
      // Turn on the rollers.
      new_superstructure_goal->intake.roller_voltage =
          9.0;  //TODO(Neil): Add real value once we have one.
    } else if (data.IsPressed(kIntakeOut)) {
      // Turn off the rollers.
      new_superstructure_goal->intake.roller_voltage =
          -9.0;  //TODO(Neil): Add real value once we have one.
    } else {
      // We don't want the rollers on.
      new_superstructure_goal->intake.roller_voltage = 0.0;
    }

    if (data.IsPressed(kArmDown)) {
      // Put the arm down to the intake level.
      new_superstructure_goal->arm_goal_position =
          1;  // TODO(Neil): Add real value once we have it.
    } else if (data.IsPressed(kArmSwitch)) {
      // Put the arm up to the level of the switch.
      new_superstructure_goal->arm_goal_position =
          1;  // TODO(Neil): Add real value once we have it.
    } else if (data.IsPressed(kArmScale)) {
      // Put the arm up to the level of the switch.
      new_superstructure_goal->arm_goal_position =
          1;  // TODO(Neil): Add real value once we have it.
    }

    if (data.IsPressed(kClawOpen)) {
      new_superstructure_goal->open_claw = true;
    } else if (data.IsPressed(kClawClose)) {
      new_superstructure_goal->open_claw = false;
    }

    if (data.IsPressed(kForkDeploy)) {
      new_superstructure_goal->deploy_fork = true;
    } else if (data.IsPressed(kForkStow)) {
      new_superstructure_goal->deploy_fork = false;
    }

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  void StartAuto() { LOG(INFO, "Starting auto mode\n"); }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  // Current goals to send to the robot.
  double intake_goal_ = 0.0;

  bool was_running_ = false;
  bool auto_running_ = false;

  double robot_velocity_ = 0.0;

  ::aos::common::actions::ActionQueue action_queue_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2018

int main() {
  ::aos::Init(-1);
  ::y2018::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
