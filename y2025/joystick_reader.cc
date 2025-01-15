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
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "frc971/input/redundant_joystick_data.h"
#include "frc971/input/swerve_joystick_input.h"
#include "frc971/zeroing/wrap.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2025/control_loops/superstructure/superstructure_goal_static.h"
#include "y2025/control_loops/superstructure/superstructure_status_static.h"

using frc971::CreateProfileParameters;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;
using y2025::control_loops::superstructure::GoalStatic;
using y2025::control_loops::superstructure::Status;

namespace y2025::input::joysticks {

namespace superstructure = y2025::control_loops::superstructure;

namespace swerve = frc971::control_loops::swerve;
// ButtonLocation constants go here
class Reader : public ::frc971::input::SwerveJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop,
         const y2025::RobotConstants *robot_constants)
      : ::frc971::input::SwerveJoystickInput(
            event_loop,
            {.vx_offset = robot_constants->input_config()->vx_offset(),
             .vy_offset = robot_constants->input_config()->vy_offset(),
             .omega_offset = robot_constants->input_config()->omega_offset(),
             .use_redundant_joysticks =
                 robot_constants->input_config()->use_redundant_joysticks()}),
        superstructure_goal_sender_(
            event_loop->MakeSender<GoalStatic>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<Status>("/superstructure")),
        robot_constants_(robot_constants) {
    CHECK(robot_constants_ != nullptr);
  }
  void AutoEnded() { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    (void)data;
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }
    aos::Sender<superstructure::GoalStatic>::StaticBuilder
        superstructure_goal_builder =
            superstructure_goal_sender_.MakeStaticBuilder();
    superstructure_goal_builder.CheckOk(superstructure_goal_builder.Send());
  }

 private:
  ::aos::Sender<GoalStatic> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  const y2025::RobotConstants *robot_constants_;
};

}  // namespace y2025::input::joysticks

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");
  frc971::constants::WaitForConstants<y2025::Constants>(&config.message());

  ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());
  frc971::constants::ConstantsFetcher<y2025::Constants> constants_fetcher(
      &constant_fetcher_event_loop);
  const y2025::RobotConstants *robot_constants =
      constants_fetcher.constants().robot();

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2025::input::joysticks::Reader reader(&event_loop, robot_constants);

  event_loop.Run();

  return 0;
}
