#include "aos/input/action_joystick_input.h"

#include "aos/input/driver_station_data.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"

using ::aos::input::driver_station::ControlBit;

namespace aos {
namespace input {

void ActionJoystickInput::RunIteration(
    const ::aos::input::driver_station::Data &data) {
  const bool last_auto_running = auto_running_;
  auto_running_ = data.GetControlBit(ControlBit::kAutonomous) &&
                  data.GetControlBit(ControlBit::kEnabled);
  if (auto_running_ != last_auto_running) {
    if (auto_running_) {
      auto_was_running_ = true;
      StartAuto();
    } else {
      StopAuto();
    }
  }

  if (!auto_running_ || (run_teleop_in_auto_ && !action_queue_.Running())) {
    if (auto_was_running_) {
      AutoEnded();
      auto_was_running_ = false;
    }
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }
    drivetrain_input_reader_->HandleDrivetrain(data);
    HandleTeleop(data);
  }

  // Process pending actions.
  action_queue_.Tick();
  was_running_ = action_queue_.Running();
}

void ActionJoystickInput::StartAuto() {
  LOG(INFO, "Starting auto mode\n");
  action_queue_.EnqueueAction(::frc971::autonomous::MakeAutonomousAction(0));
}

void ActionJoystickInput::StopAuto() {
  LOG(INFO, "Stopping auto mode\n");
  action_queue_.CancelAllActions();
}

}  // namespace input
}  // namespace aos
