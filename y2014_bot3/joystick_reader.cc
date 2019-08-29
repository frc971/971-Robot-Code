#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/init.h"
#include "aos/input/joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "aos/util/log_interval.h"
#include "aos/time/time.h"

#include "aos/input/drivetrain_input.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2014_bot3/control_loops/rollers/rollers_goal_generated.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::DrivetrainInputReader;

namespace y2014_bot3 {
namespace input {
namespace joysticks {

// Joystick & button addresses.
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 3), kShiftHigh2(2, 2), kShiftLow(2, 1);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kTurn1(1, 7);
const ButtonLocation kTurn2(1, 11);

const ButtonLocation kFrontRollersIn(3, 8);
const ButtonLocation kBackRollersIn(3, 7);
const ButtonLocation kFrontRollersOut(3, 6);
const ButtonLocation kBackRollersOut(4, 12);
const ButtonLocation kHumanPlayer(4, 11);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::JoystickInput(event_loop),
        rollers_goal_sender_(
            event_loop->MakeSender<::y2014_bot3::control_loops::rollers::Goal>(
                "/rollers")),
        autonomous_action_factory_(
            ::frc971::autonomous::BaseAutonomousActor::MakeFactory(
                event_loop)) {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        event_loop, DrivetrainInputReader::InputType::kSteeringWheel,
        ::y2014_bot3::control_loops::drivetrain::GetDrivetrainConfig());
  }

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
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

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }

    action_queue_.Tick();
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      AOS_LOG(DEBUG, "Canceling\n");
    }

    // Rollers.
    auto builder = rollers_goal_sender_.MakeBuilder();
    control_loops::rollers::GoalT rollers_goal;
    if (data.IsPressed(kFrontRollersIn)) {
      rollers_goal.intake = 1;
    } else if (data.IsPressed(kFrontRollersOut)) {
      rollers_goal.low_spit = 1;
    } else if (data.IsPressed(kBackRollersIn)) {
      rollers_goal.intake = -1;
    } else if (data.IsPressed(kBackRollersOut)) {
      rollers_goal.low_spit = -1;
    } else if (data.IsPressed(kHumanPlayer)) {
      rollers_goal.human_player = true;
    }
    if (!builder.Send(control_loops::rollers::Goal::Pack(*builder.fbb(),
                                                         &rollers_goal))) {
      AOS_LOG(WARNING, "Sending rollers values failed.\n");
    }
  }

 private:
  void StartAuto() {
    AOS_LOG(INFO, "Starting auto mode.\n");
    ::frc971::autonomous::AutonomousActionParamsT params;
    params.mode = 0;
    action_queue_.EnqueueAction(autonomous_action_factory_.Make(params));
  }

  void StopAuto() {
    AOS_LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  bool auto_running_ = false;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::std::chrono::milliseconds(200), WARNING,
                                     "no drivetrain status");

  ::aos::common::actions::ActionQueue action_queue_;

  ::std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;
  ::aos::Sender<::y2014_bot3::control_loops::rollers::Goal>
      rollers_goal_sender_;

  ::frc971::autonomous::BaseAutonomousActor::Factory autonomous_action_factory_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2014_bot3

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2014_bot3::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
