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
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/input/redundant_joystick_data.h"
#include "frc971/input/swerve_joystick_input.h"
#include "y2024_swerve/constants/constants_generated.h"

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
  Reader(::aos::EventLoop *event_loop, const RobotConstants *robot_constants)
      : ::frc971::input::SwerveJoystickInput(
            event_loop,
            {.vx_offset = robot_constants->input_config()->vx_offset(),
             .vy_offset = robot_constants->input_config()->vy_offset(),
             .omega_offset = robot_constants->input_config()->omega_offset(),
             .use_redundant_joysticks =
                 robot_constants->input_config()->use_redundant_joysticks()}) {}

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
  frc971::constants::ConstantsFetcher<y2024_swerve::Constants>
      constants_fetcher(&event_loop);

  const y2024_swerve::RobotConstants *robot_constants =
      constants_fetcher.constants().robot();

  ::y2024_swerve::input::joysticks::Reader reader(&event_loop, robot_constants);

  event_loop.Run();

  return 0;
}
