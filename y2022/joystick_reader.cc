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
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/input/action_joystick_input.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/drivetrain_input.h"
#include "frc971/input/joystick_input.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;

namespace y2022 {
namespace input {
namespace joysticks {

const ButtonLocation kCatapultPos(3, 3);
const ButtonLocation kFire(3, 1);

namespace superstructure = y2022::control_loops::superstructure;

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2022::control_loops::drivetrain::GetDrivetrainConfig(),
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

    aos::Sender<superstructure::Goal>::Builder builder =
        superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters> catapult_profile =
        frc971::CreateProfileParameters(*builder.fbb(), 5.0, 30.0);

    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder
        catapult_return_builder =
            builder.MakeBuilder<StaticZeroingSingleDOFProfiledSubsystemGoal>();
    catapult_return_builder.add_unsafe_goal(
        data.IsPressed(kCatapultPos) ? 0.3 : -0.85);
    catapult_return_builder.add_profile_params(catapult_profile);
    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        catapult_return_offset = catapult_return_builder.Finish();

    superstructure::CatapultGoal::Builder catapult_builder =
        builder.MakeBuilder<superstructure::CatapultGoal>();
    catapult_builder.add_return_position(catapult_return_offset);
    catapult_builder.add_fire(data.IsPressed(kFire));
    catapult_builder.add_shot_position(0.3);
    catapult_builder.add_shot_velocity(15.0);
    flatbuffers::Offset<superstructure::CatapultGoal> catapult_offset =
        catapult_builder.Finish();

    superstructure::Goal::Builder goal_builder =
        builder.MakeBuilder<superstructure::Goal>();
    goal_builder.add_catapult(catapult_offset);

    if (builder.Send(goal_builder.Finish()) != aos::RawSender::Error::kOk) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2022

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2022::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
