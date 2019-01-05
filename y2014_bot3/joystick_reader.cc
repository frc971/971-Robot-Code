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
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2014_bot3::control_loops::rollers_queue;
using ::frc971::sensors::gyro_reading;

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
  Reader() {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kSteeringWheel,
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
      LOG(DEBUG, "Canceling\n");
    }

    // Rollers.
    auto rollers_goal = control_loops::rollers_queue.goal.MakeMessage();
    rollers_goal->Zero();
    if (data.IsPressed(kFrontRollersIn)) {
      rollers_goal->intake = 1;
    } else if (data.IsPressed(kFrontRollersOut)) {
      rollers_goal->low_spit = 1;
    } else if (data.IsPressed(kBackRollersIn)) {
      rollers_goal->intake = -1;
    } else if (data.IsPressed(kBackRollersOut)) {
      rollers_goal->low_spit = -1;
    } else if (data.IsPressed(kHumanPlayer)) {
      rollers_goal->human_player = true;
    }
    if (!rollers_goal.Send()) {
      LOG(WARNING, "Sending rollers values failed.\n");
    }
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode.\n");
    ::frc971::autonomous::AutonomousActionParams params;
    params.mode = 0;
    action_queue_.EnqueueAction(
        ::frc971::autonomous::MakeAutonomousAction(params));
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  bool auto_running_ = false;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::std::chrono::milliseconds(200), WARNING,
                                     "no drivetrain status");

  ::aos::common::actions::ActionQueue action_queue_;

  ::std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2014_bot3

int main() {
  ::aos::Init(-1);
  ::y2014_bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
