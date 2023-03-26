#include "frc971/input/action_joystick_input.h"

#include "frc971/autonomous/auto_generated.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/redundant_joystick_data.h"

using ::frc971::input::driver_station::ControlBit;

namespace frc971 {
namespace input {

void ActionJoystickInput::RunIteration(
    const ::frc971::input::driver_station::Data &unsorted_data) {
  if (input_config_.use_redundant_joysticks) {
    driver_station::RedundantData redundant_data_storage(unsorted_data);
    DoRunIteration(redundant_data_storage);
  } else {
    DoRunIteration(unsorted_data);
  }
}

void ActionJoystickInput::DoRunIteration(
    const ::frc971::input::driver_station::Data &data) {
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
}  // namespace frc971
