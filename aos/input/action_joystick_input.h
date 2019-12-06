#ifndef AOS_INPUT_ACTION_JOYSTICK_INPUT_H_
#define AOS_INPUT_ACTION_JOYSTICK_INPUT_H_

#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "frc971/autonomous/auto_generated.h"
#include "frc971/autonomous/auto_mode_generated.h"
#include "frc971/autonomous/base_autonomous_actor.h"

namespace aos {
namespace input {

// Class to abstract out managing actions, autonomous mode, and drivetrains.
// Turns out we do the same thing every year, so let's stop copying it.
class ActionJoystickInput : public ::aos::input::JoystickInput {
 public:
   // Configuration parameters that don't really belong in the DrivetrainConfig.
  struct InputConfig {
    // If true, we will run teleop during auto mode after auto mode ends.  This
    // is to support the 2019 sandstorm mode.  Auto will run, and then when the
    // action ends (either when it's done, or when the driver triggers it to
    // finish early), we will run teleop regardless of the mode.
    bool run_teleop_in_auto = false;
    // A button, for use with the run_teleop_in_auto, that will cancel the auto
    // mode, and if run_telop_in_auto is specified, resume teloperation.
    const driver_station::ButtonLocation cancel_auto_button = {-1, -1};
  };
  ActionJoystickInput(
      ::aos::EventLoop *event_loop,
      const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
          &dt_config,
      DrivetrainInputReader::InputType input_type,
      const InputConfig &input_config)
      : ::aos::input::JoystickInput(event_loop),
        input_config_(input_config),
        drivetrain_input_reader_(
            DrivetrainInputReader::Make(event_loop, input_type, dt_config)),
        dt_config_(dt_config),
        autonomous_action_factory_(
            ::frc971::autonomous::BaseAutonomousActor::MakeFactory(event_loop)),
        autonomous_mode_fetcher_(
            event_loop->MakeFetcher<::frc971::autonomous::AutonomousMode>(
                "/autonomous")) {}

  virtual ~ActionJoystickInput() {}

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

  // Returns the current robot velocity.
  double robot_velocity() const {
    return drivetrain_input_reader_->robot_velocity();
  }

  // Returns the drivetrain config.
  const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
  dt_config() const {
    return dt_config_;
  }

  // Sets the vision align function.  This function runs before the normal
  // drivetrain code runs.  If it returns true, we are in vision align mode and
  // no drivetain code is run.  If it returns false, the vision align function
  // is assumed to be disabled and normal drive code is run.
  void set_vision_align_fn(
      ::std::function<bool(const ::aos::input::driver_station::Data &data)>
          vision_align_fn) {
    drivetrain_input_reader_->set_vision_align_fn(vision_align_fn);
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
  virtual uint32_t GetAutonomousMode() {
    autonomous_mode_fetcher_.Fetch();
    if (autonomous_mode_fetcher_.get() == nullptr) {
      AOS_LOG(WARNING, "no auto mode values\n");
      return 0;
    }
    return autonomous_mode_fetcher_->mode();
  }

  // True if the internal state machine thinks auto is running right now.
  bool auto_running_ = false;
  // True if we think that the auto *action* is running right now.
  bool auto_action_running_ = false;
  // True if an action was running last cycle.
  bool was_running_ = false;

  const InputConfig input_config_;

  // Bool to track if auto was running the last cycle through.  This lets us
  // call AutoEnded when the auto mode function stops.
  bool auto_was_running_ = false;
  ::std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;
  ::aos::common::actions::ActionQueue action_queue_;

  const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
      dt_config_;

  ::frc971::autonomous::BaseAutonomousActor::Factory autonomous_action_factory_;

  ::aos::Fetcher<::frc971::autonomous::AutonomousMode> autonomous_mode_fetcher_;
};

}  // namespace input
}  // namespace aos

#endif  // AOS_INPUT_ACTION_JOYSTICK_INPUT_H_
