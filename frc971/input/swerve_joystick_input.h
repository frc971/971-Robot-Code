#ifndef AOS_INPUT_SWERVE_JOYSTICK_INPUT_H_
#define AOS_INPUT_SWERVE_JOYSTICK_INPUT_H_

#include "aos/actions/actions.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"

using frc971::control_loops::drivetrain::PistolBottomButtonUse;
using frc971::control_loops::drivetrain::PistolSecondButtonUse;
using frc971::control_loops::drivetrain::PistolTopButtonUse;

namespace frc971::input {

class SwerveJoystickInput : public ::frc971::input::JoystickInput {
 public:
  // Configuration parameters that don't really belong in the DrivetrainConfig.
  struct InputConfig {
    // Use button 14 and 15 to encode the id of the joystick and remap the
    // joysticks so that their ids are independent of their order on the
    // driverstation.
    bool use_redundant_joysticks = false;
  };
  SwerveJoystickInput(::aos::EventLoop *event_loop,
                      const InputConfig &input_config)
      : ::frc971::input::JoystickInput(event_loop),
        input_config_(input_config),
        drivetrain_input_reader_(SwerveDrivetrainInputReader::Make(event_loop)),
        goal_sender_(event_loop->MakeSender<control_loops::swerve::GoalStatic>(
            "/drivetrain")) {}

  virtual ~SwerveJoystickInput() {}

 protected:
  bool was_running_action() { return was_running_; }

  // Returns true if an action is running.
  bool ActionRunning() { return action_queue_.Running(); }
  // Cancels all actions.
  void CancelAllActions() { action_queue_.CancelAllActions(); }
  // Cancels the current action.
  void CancelCurrentAction() { action_queue_.CancelCurrentAction(); }

  // Enqueues an action.
  void EnqueueAction(::std::unique_ptr<::aos::common::actions::Action> action) {
    action_queue_.EnqueueAction(::std::move(action));
  }

 private:
  // Handles any year specific superstructure code.
  virtual void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) = 0;

  void RunIteration(const ::frc971::input::driver_station::Data &data) override;

  void DoRunIteration(const ::frc971::input::driver_station::Data &data);

  void HandleDrivetrain(const ::frc971::input::driver_station::Data &data);

  // True if an action was running last cycle.
  bool was_running_ = false;

  // Bool to track if auto was running the last cycle through.  This lets us
  // call AutoEnded when the auto mode function stops.

  const InputConfig input_config_;

  ::std::unique_ptr<SwerveDrivetrainInputReader> drivetrain_input_reader_;
  ::aos::Sender<control_loops::swerve::GoalStatic> goal_sender_;

  ::aos::common::actions::ActionQueue action_queue_;
};

}  // namespace frc971::input

#endif  // AOS_SWERVE_ACTION_JOYSTICK_INPUT_H_
