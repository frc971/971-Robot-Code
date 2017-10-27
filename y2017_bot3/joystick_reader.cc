#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/common/actions/actions.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/linux_code/init.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017_bot3/control_loops/superstructure/superstructure.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2017_bot3::control_loops::superstructure_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

namespace y2017_bot3 {
namespace input {
namespace joysticks {

const ButtonLocation kHangerOn(3, 1);
const ButtonLocation kRollersOn(3, 2);
const ButtonLocation kGearOut(3, 3);

const ButtonLocation kRollerOn(3, 4);

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kSteeringWheel);
  }

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    const bool last_auto_running = auto_running_;
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

    // Process pending actions.
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
    robot_velocity_ = drivetrain_input_reader_->robot_velocity();
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
    new_superstructure_goal->voltage_rollers = 0.0;

    if (data.IsPressed(kRollerOn)) {
      new_superstructure_goal->voltage_rollers = 12.0;
    }

    if (data.IsPressed(kHangerOn)) {
      new_superstructure_goal->hanger_voltage = 12.0;
    }


    if (data.IsPressed(kGearOut)) {
      new_superstructure_goal->fingers_out = true;
    } else {
      new_superstructure_goal->fingers_out = false;
    }

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
  double robot_velocity_ = 0.0;
  ::aos::common::actions::ActionQueue action_queue_;

  bool was_running_ = false;
  bool auto_running_ = false;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2017_bot3

int main() {
  ::aos::Init(-1);
  ::y2017_bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
