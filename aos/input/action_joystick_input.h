#ifndef AOS_INPUT_ACTION_JOYSTICK_INPUT_H_
#define AOS_INPUT_ACTION_JOYSTICK_INPUT_H_

#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"

namespace aos {
namespace input {

// Class to abstract out managing actions, autonomous mode, and drivetrains.
// Turns out we do the same thing every year, so let's stop copying it.
class ActionJoystickInput : public ::aos::input::JoystickInput {
 public:
  ActionJoystickInput(
      ::aos::EventLoop *event_loop,
      const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
          &dt_config)
      : ::aos::input::JoystickInput(event_loop),
        drivetrain_input_reader_(DrivetrainInputReader::Make(
            DrivetrainInputReader::InputType::kPistol, dt_config)) {}

  virtual ~ActionJoystickInput() {}

 private:
  // Handles any year specific superstructure code.
  virtual void HandleTeleop(const ::aos::input::driver_station::Data &data) = 0;

  void RunIteration(const ::aos::input::driver_station::Data &data) override;

  void StartAuto();
  void StopAuto();

  // True if the internal state machine thinks auto is running right now.
  bool auto_running_ = false;
  // True if an action was running last cycle.
  bool was_running_ = false;

  ::std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;
  ::aos::common::actions::ActionQueue action_queue_;
};

}  // namespace input
}  // namespace aos

#endif  // AOS_INPUT_ACTION_JOYSTICK_INPUT_H_
