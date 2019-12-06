#include "aos/input/action_joystick_input.h"

#include "aos/input/driver_station_data.h"
#include "frc971/autonomous/auto_generated.h"
#include "frc971/autonomous/auto_mode_generated.h"
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

  if (!auto_running_ ||
      (input_config_.run_teleop_in_auto && !action_queue_.Running())) {
    if (auto_was_running_) {
      AutoEnded();
      auto_was_running_ = false;
    }
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      AOS_LOG(DEBUG, "Canceling\n");
    }
    drivetrain_input_reader_->HandleDrivetrain(data);
    HandleTeleop(data);
  }

  if (auto_action_running_ &&
      data.IsPressed(input_config_.cancel_auto_button)) {
    StopAuto();
  }

  // Process pending actions.
  action_queue_.Tick();
  was_running_ = action_queue_.Running();
}

void ActionJoystickInput::StartAuto() {
  AOS_LOG(INFO, "Starting auto mode\n");
  frc971::autonomous::AutonomousActionParamsT params;
  params.mode = GetAutonomousMode();

  action_queue_.EnqueueAction(autonomous_action_factory_.Make(params));
  auto_action_running_ = true;
}

void ActionJoystickInput::StopAuto() {
  AOS_LOG(INFO, "Stopping auto mode\n");
  action_queue_.CancelAllActions();
  auto_action_running_ = false;
  AutoEnded();
}

}  // namespace input
}  // namespace aos
