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

 protected:
  void set_run_teleop_in_auto(bool run_teleop_in_auto) {
    run_teleop_in_auto_ = run_teleop_in_auto;
  }

 private:
  // Handles anything that needs to be cleaned up when the auto action exits.
  virtual void AutoEnded() {}
  // Handles any year specific superstructure code.
  virtual void HandleTeleop(const ::aos::input::driver_station::Data &data) = 0;

  void RunIteration(const ::aos::input::driver_station::Data &data) override;

  void StartAuto();
  void StopAuto();

  // Returns the current autonomous mode which has been selected by robot
  // inputs.
  virtual uint32_t GetAutonomousMode() { return 0; }

  // True if the internal state machine thinks auto is running right now.
  bool auto_running_ = false;
  // True if an action was running last cycle.
  bool was_running_ = false;

  // If true, we will run teleop during auto mode after auto mode ends.  This is
  // to support the 2019 sandstorm mode.  Auto will run, and then when the
  // action ends (either when it's done, or when the driver triggers it to
  // finish early), we will run teleop regardless of the mode.
  bool run_teleop_in_auto_ = false;

  // Bool to track if auto was running the last cycle through.  This lets us
  // call AutoEnded when the auto mode function stops.
  bool auto_was_running_ = false;

  ::std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;
  ::aos::common::actions::ActionQueue action_queue_;
};

}  // namespace input
}  // namespace aos

#endif  // AOS_INPUT_ACTION_JOYSTICK_INPUT_H_
