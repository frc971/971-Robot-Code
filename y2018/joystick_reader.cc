#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/common/actions/actions.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/log_interval.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/linux_code/init.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::DrivetrainInputReader;

namespace y2018 {
namespace input {
namespace joysticks {

std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader_;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {
    drivetrain_input_reader_ = DrivetrainInputReader::Make(
        DrivetrainInputReader::InputType::kSteeringWheel,
        ::y2018::control_loops::drivetrain::GetDrivetrainConfig());
  }

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
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

    if (!auto_running_) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }

    // Process any pending actions.
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    drivetrain_input_reader_->HandleDrivetrain(data);
    robot_velocity_ = drivetrain_input_reader_->robot_velocity();
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }
  }

 private:
  void StartAuto() { LOG(INFO, "Starting auto mode\n"); }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  bool was_running_ = false;
  bool auto_running_ = false;

  double robot_velocity_ = 0.0;

  ::aos::common::actions::ActionQueue action_queue_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2018

int main() {
  ::aos::Init(-1);
  ::y2018::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
