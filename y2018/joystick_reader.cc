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
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
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

namespace arm = ::y2018::control_loops::superstructure::arm;

const ButtonLocation kIntakeClosed(4, 1);

const ButtonLocation kIntakeIn(3, 16);
const ButtonLocation kIntakeOut(4, 3);

const ButtonLocation kArmFrontHighBox(4, 5);
const ButtonLocation kArmFrontExtraHighBox(3, 8);
const ButtonLocation kArmFrontMiddle2Box(4, 7);
const ButtonLocation kArmFrontMiddle1Box(4, 9);
const ButtonLocation kArmFrontLowBox(4, 11);
const ButtonLocation kArmFrontSwitch(3, 14);

const ButtonLocation kArmBackHighBox(4, 6);
const ButtonLocation kArmBackExtraHighBox(3, 10);
const ButtonLocation kArmBackMiddle2Box(4, 8);
const ButtonLocation kArmBackMiddle1Box(4, 10);
const ButtonLocation kArmBackLowBox(4, 12);
const ButtonLocation kArmBackSwitch(3, 12);

const ButtonLocation kArmNeutral(3, 13);
const ButtonLocation kArmUp(3, 9);

const ButtonLocation kArmPickupBoxFromIntake(3, 6);

const ButtonLocation kClawOpen(3, 1);

const ButtonLocation kForkDeploy(3, 11);
const ButtonLocation kForkStow(3, 10);

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kPistol,
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

    superstructure_queue.position.FetchLatest();
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get() ||
        !superstructure_queue.position.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    if (data.IsPressed(kIntakeClosed)) {
      // Deploy the intake.
      if (superstructure_queue.position->box_back_beambreak_triggered) {
        intake_goal_ = 0.30;
      } else {
        intake_goal_ = 0.07;
      }
    } else {
      // Bring in the intake.
      intake_goal_ = -3.3;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    new_superstructure_goal->intake.left_intake_angle = intake_goal_;
    new_superstructure_goal->intake.right_intake_angle = intake_goal_;

    if (data.IsPressed(kIntakeIn) || data.IsPressed(kArmPickupBoxFromIntake)) {
      // Turn on the rollers.
      new_superstructure_goal->intake.roller_voltage = 8.0;
    } else if (data.IsPressed(kIntakeOut)) {
      // Turn off the rollers.
      new_superstructure_goal->intake.roller_voltage = -12.0;
    } else {
      // We don't want the rollers on.
      new_superstructure_goal->intake.roller_voltage = 0.0;
    }

    // If we are disabled, stay at the node closest to where we start.  This
    // should remove long motions when enabled.
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      arm_goal_position_ = superstructure_queue.status->arm.current_node;
    }

    bool grab_box = false;
    if (data.IsPressed(kArmPickupBoxFromIntake)) {
      arm_goal_position_ = arm::NeutralIndex();
      grab_box = true;
    } else if (data.IsPressed(kArmNeutral)) {
      arm_goal_position_ = arm::NeutralIndex();
    } else if (data.IsPressed(kArmUp)) {
      arm_goal_position_ = arm::UpIndex();
    } else if (data.IsPressed(kArmFrontSwitch)) {
      arm_goal_position_ = arm::FrontSwitchIndex();
    } else if (data.IsPressed(kArmFrontHighBox) ||
               data.IsPressed(kArmFrontExtraHighBox)) {
      arm_goal_position_ = arm::FrontHighBoxIndex();
    } else if (data.IsPressed(kArmFrontMiddle2Box)) {
      arm_goal_position_ = arm::FrontMiddle2BoxIndex();
    } else if (data.IsPressed(kArmFrontMiddle1Box)) {
      arm_goal_position_ = arm::FrontMiddle1BoxIndex();
    } else if (data.IsPressed(kArmFrontLowBox)) {
      arm_goal_position_ = arm::FrontLowBoxIndex();
    } else if (data.IsPressed(kArmBackHighBox) ||
               data.IsPressed(kArmBackExtraHighBox)) {
      arm_goal_position_ = arm::BackHighBoxIndex();
    } else if (data.IsPressed(kArmBackMiddle2Box)) {
      arm_goal_position_ = arm::BackMiddle2BoxIndex();
    } else if (data.IsPressed(kArmBackMiddle1Box)) {
      arm_goal_position_ = arm::BackMiddle1BoxIndex();
    } else if (data.IsPressed(kArmBackLowBox)) {
      arm_goal_position_ = arm::BackLowBoxIndex();
    }  else if (data.IsPressed(kArmBackSwitch)) {
      arm_goal_position_ = arm::BackSwitchIndex();
    }

    new_superstructure_goal->arm_goal_position = arm_goal_position_;

    if (data.IsPressed(kClawOpen) || data.PosEdge(kArmPickupBoxFromIntake)) {
      new_superstructure_goal->open_claw = true;
    } else {
      new_superstructure_goal->open_claw = false;
    }

    if (data.IsPressed(kForkDeploy)) {
      new_superstructure_goal->deploy_fork = true;
    } else if (data.IsPressed(kForkStow)) {
      new_superstructure_goal->deploy_fork = false;
    }
    new_superstructure_goal->grab_box = grab_box;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");

    ::frc971::autonomous::AutonomousActionParams params;
    ::frc971::autonomous::auto_mode.FetchLatest();
    if (::frc971::autonomous::auto_mode.get() != nullptr) {
      params.mode = ::frc971::autonomous::auto_mode->mode;
    } else {
      LOG(WARNING, "no auto mode values\n");
      params.mode = 0;
    }
    action_queue_.EnqueueAction(
        ::frc971::autonomous::MakeAutonomousAction(params));
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  // Current goals to send to the robot.
  double intake_goal_ = -M_PI * 2.0 / 3.0;

  bool was_running_ = false;
  bool auto_running_ = false;

  double robot_velocity_ = 0.0;

  int arm_goal_position_ = 0;

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
