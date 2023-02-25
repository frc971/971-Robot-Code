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
#include "frc971/zeroing/wrap.h"
#include "y2023/constants.h"
#include "y2023/control_loops/drivetrain/drivetrain_base.h"
#include "y2023/control_loops/superstructure/arm/generated_graph.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

using frc971::CreateProfileParameters;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::input::driver_station::ButtonLocation;
using frc971::input::driver_station::ControlBit;
using frc971::input::driver_station::JoystickAxis;
using frc971::input::driver_station::POVLocation;
using y2023::control_loops::superstructure::RollerGoal;

namespace y2023 {
namespace input {
namespace joysticks {

// TODO(milind): add correct locations
const ButtonLocation kIntake(4, 5);
const ButtonLocation kScore(4, 4);
const ButtonLocation kSpit(4, 13);

const ButtonLocation kSuck(4, 12);

const ButtonLocation kWrist(4, 10);

namespace superstructure = y2023::control_loops::superstructure;
namespace arm = superstructure::arm;

class Reader : public ::frc971::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::frc971::input::ActionJoystickInput(
            event_loop,
            ::y2023::control_loops::drivetrain::GetDrivetrainConfig(),
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

    RollerGoal roller_goal = RollerGoal::IDLE;

    // TODO(milind): add more actions and paths
    if (data.IsPressed(kIntake)) {
      arm_goal_position_ = arm::ScorePosIndex();
    } else if (data.IsPressed(kScore)) {
      arm_goal_position_ = arm::ScorePosIndex();
    } else {
      arm_goal_position_ = arm::NeutralPosIndex();
    }

    if (data.IsPressed(kSuck)) {
      roller_goal = RollerGoal::INTAKE;
    } else if (data.IsPressed(kSpit)) {
      roller_goal = RollerGoal::SPIT;
    }

    double wrist_goal = 0.1;

    if (data.IsPressed(kWrist)) {
      wrist_goal = 1.5;
    } else {
      wrist_goal = 0.1;
    }

    {
      auto builder = superstructure_goal_sender_.MakeBuilder();

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          wrist_offset =
              CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
                  *builder.fbb(), wrist_goal,
                  CreateProfileParameters(*builder.fbb(), 12.0, 90.0));

      superstructure::Goal::Builder superstructure_goal_builder =
          builder.MakeBuilder<superstructure::Goal>();
      superstructure_goal_builder.add_arm_goal_position(arm_goal_position_);
      superstructure_goal_builder.add_roller_goal(roller_goal);
      superstructure_goal_builder.add_wrist(wrist_offset);
      if (builder.Send(superstructure_goal_builder.Finish()) !=
          aos::RawSender::Error::kOk) {
        AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
      }
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;

  uint32_t arm_goal_position_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2023

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2023::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  return 0;
}
