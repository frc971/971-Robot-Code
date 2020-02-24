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
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

using aos::input::driver_station::ButtonLocation;
using aos::input::driver_station::ControlBit;
using aos::input::driver_station::JoystickAxis;
using aos::input::driver_station::POVLocation;

using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;


namespace y2020 {
namespace input {
namespace joysticks {

namespace superstructure = y2020::control_loops::superstructure;

// TODO(sabina): fix button locations.

const ButtonLocation kShootFast(4, 1);
const ButtonLocation kShootSlow(4, 2);
const ButtonLocation kIntakeExtend(4, 3);
const ButtonLocation kIntakeIn(4, 4);
const ButtonLocation kSpit(4, 5);

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2020::control_loops::drivetrain::GetDrivetrainConfig(),
            ::aos::input::DrivetrainInputReader::InputType::kPistol, {}),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>(
                "/superstructure")) {}

  void AutoEnded() override {
    AOS_LOG(INFO, "Auto ended, assuming disc and have piece\n");
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    double hood_pos = constants::Values::kHoodRange().upper;
    double intake_pos = constants::Values::kIntakeRange().lower;
    double turret_pos = 0.0;
    float roller_speed = 0.0f;
    double accelerator_speed = 0.0;
    double finisher_speed = 0.0;

    if (data.IsPressed(kShootFast)) {
      accelerator_speed = 300.0;
      finisher_speed = 300.0;
    }

    else if (data.IsPressed(kShootSlow)) {
      accelerator_speed = 30.0;
      finisher_speed = 30.0;
    }

    if (data.IsPressed(kIntakeExtend)) {
      intake_pos = constants::Values::kIntakeRange().middle();
    }

    if (data.IsPressed(kIntakeIn)) {
      roller_speed = 6.0f;
    } else if (data.IsPressed(kSpit)) {
      roller_speed = -6.0f;
    }

    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<superstructure::Goal> superstructure_goal_offset;
    {
      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          hood_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), hood_pos);

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), intake_pos);

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          turret_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), turret_pos);

      flatbuffers::Offset<superstructure::ShooterGoal> shooter_offset =
          superstructure::CreateShooterGoal(*builder.fbb(), accelerator_speed, finisher_speed);

      superstructure::Goal::Builder superstructure_goal_builder =
          builder.MakeBuilder<superstructure::Goal>();

      superstructure_goal_builder.add_hood(hood_offset);
      superstructure_goal_builder.add_intake(intake_offset);
      superstructure_goal_builder.add_turret(turret_offset);
      superstructure_goal_builder.add_roller_voltage(roller_speed);
      superstructure_goal_builder.add_shooter(shooter_offset);

      if (!builder.Send(superstructure_goal_builder.Finish())) {
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
}  // namespace y2020

int main() {
  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2020::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
