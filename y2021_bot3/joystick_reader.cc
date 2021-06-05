#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "y2021_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2021_bot3/control_loops/superstructure/superstructure_status_generated.h"

using aos::input::driver_station::ButtonLocation;
using aos::input::driver_station::ControlBit;
using aos::input::driver_station::JoystickAxis;
using aos::input::driver_station::POVLocation;

namespace y2021_bot3 {
namespace input {
namespace joysticks {

namespace superstructure = y2021_bot3::control_loops::superstructure;

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2021_bot3::control_loops::drivetrain::GetDrivetrainConfig(),
            ::aos::input::DrivetrainInputReader::InputType::kPistol, {}),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>(
                "/superstructure")) {}

  void AutoEnded() override { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::aos::input::driver_station::Data & /*data*/) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2021_bot3

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2021_bot3::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
