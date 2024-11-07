#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "absl/flags/flag.h"

#include "aos/actions/actions.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/input/redundant_joystick_data.h"
#include "frc971/input/swerve_joystick_input.h"

using frc971::CreateProfileParameters;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;
using Side = frc971::control_loops::drivetrain::RobotSide;

namespace y2024_swerve::input::joysticks {

namespace swerve = frc971::control_loops::swerve;

class Reader : public ::frc971::input::SwerveJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::SwerveJoystickInput(
            event_loop, {.use_redundant_joysticks = true}) {}

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    // Where teleop logic will eventually go when there is superstructure code
    (void)data;
  }
};

}  // namespace y2024_swerve::input::joysticks

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024_swerve::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
