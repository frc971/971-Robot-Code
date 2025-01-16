#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "absl/flags/flag.h"

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/input/redundant_joystick_data.h"
#include "frc971/input/swerve_joystick_input.h"
#include "frc971/zeroing/wrap.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_goal_static.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_status_static.h"

using frc971::CreateProfileParameters;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;

namespace y2024_bot3::input::joysticks {

namespace superstructure = y2024_bot3::control_loops::superstructure;

// ButtonLocation constants go here

class Reader : public ::frc971::input::SwerveJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop,
         const y2024_bot3::RobotConstants *robot_constants)
      : ::frc971::input::SwerveJoystickInput(
            event_loop,
            {.vx_offset = robot_constants->input_config()->vx_offset(),
             .vy_offset = robot_constants->input_config()->vy_offset(),
             .omega_offset = robot_constants->input_config()->omega_offset(),
             .use_redundant_joysticks =
                 robot_constants->input_config()->use_redundant_joysticks()})

  {}
  void AutoEnded() { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    (void)data;
  }
};
}  // namespace y2024_bot3::input::joysticks

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");
  frc971::constants::WaitForConstants<y2024_bot3::Constants>(&config.message());

  ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());
  frc971::constants::ConstantsFetcher<y2024_bot3::Constants> constants_fetcher(
      &constant_fetcher_event_loop);
  const y2024_bot3::RobotConstants *robot_constants =
      constants_fetcher.constants().robot();

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024_bot3::input::joysticks::Reader reader(&event_loop, robot_constants);

  event_loop.Run();

  return 0;
}
