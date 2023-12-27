#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/input/redundant_joystick_data.h"
#include "frc971/zeroing/wrap.h"
#include "y2024_defense/constants.h"
#include "y2024_defense/control_loops/drivetrain/drivetrain_base.h"

using Side = frc971::control_loops::drivetrain::RobotSide;

namespace y2024_defense {
namespace input {
namespace joysticks {

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2024_defense::control_loops::drivetrain::GetDrivetrainConfig(),
            ::frc971::input::DrivetrainInputReader::InputType::kPistol,
            {.use_redundant_joysticks = true}) {}

  void AutoEnded() override { AOS_LOG(INFO, "Auto ended.\n"); }

  bool has_scored_ = false;

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    (void)data;
  }
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2024_defense

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024_defense::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
