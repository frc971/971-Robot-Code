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
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_status_generated.h"

using frc971::CreateProfileParameters;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;

namespace y2022_bot3 {
namespace input {
namespace joysticks {

namespace superstructure = y2022_bot3::control_loops::superstructure;

// TODO(niko): add a climber button
const ButtonLocation kIntake(4, 10);
const ButtonLocation kSpit(4, 9);

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2022_bot3::control_loops::drivetrain::GetDrivetrainConfig(),
            ::frc971::input::DrivetrainInputReader::InputType::kPistol, {}),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>(
                "/superstructure")) {}

  void AutoEnded() override { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }
    constexpr double kIntakeOutPosition = 0.0;
    constexpr double kIntakeInPosition = 1.47;

    double roller_speed = 0.0;
    double intake_pos = kIntakeInPosition;

    if (data.IsPressed(kIntake) || data.IsPressed(kSpit)) {
      intake_pos = kIntakeOutPosition;

      if (data.IsPressed(kIntake)) {
        roller_speed = 12.0;
      } else {
        roller_speed = -12.0;
      }
    }

    {
      auto builder = superstructure_goal_sender_.MakeBuilder();

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), intake_pos,
              CreateProfileParameters(*builder.fbb(), 8.0, 40.0));

      superstructure::Goal::Builder superstructure_goal_builder =
          builder.MakeBuilder<superstructure::Goal>();

      superstructure_goal_builder.add_intake(intake_offset);
      superstructure_goal_builder.add_roller_speed(roller_speed);

      if (builder.Send(superstructure_goal_builder.Finish()) !=
          aos::RawSender::Error::kOk) {
        AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
      }
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2022_bot3

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2022_bot3::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
