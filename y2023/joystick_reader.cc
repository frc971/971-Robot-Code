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

const ButtonLocation kMidBackTipConeScoreLeft(4, 15);
const ButtonLocation kHighBackTipConeScoreLeft(4, 14);
const ButtonLocation kMidBackTipConeScoreRight(3, 2);

const ButtonLocation kGroundPickupConeUp(4, 7);
const ButtonLocation kGroundPickupConeDown(4, 8);
const ButtonLocation kHPConePickup(4, 6);

const ButtonLocation kSuck(4, 12);

const ButtonLocation kWrist(4, 10);

namespace superstructure = y2023::control_loops::superstructure;
namespace arm = superstructure::arm;

enum class GamePiece {
  CONE_UP = 0,
  CONE_DOWN = 1,
  CUBE = 2,
};

struct ArmSetpoint {
  uint32_t index;
  double wrist_goal;
  std::optional<double> score_wrist_goal = std::nullopt;
  GamePiece game_piece;
  ButtonLocation button;
};

const std::vector<ArmSetpoint> setpoints = {
    {
        .index = arm::GroundPickupBackConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .button = kGroundPickupConeUp,
    },
    {
        .index = arm::GroundPickupBackConeDownIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .button = kGroundPickupConeDown,
    },
    {
        .index = arm::ScoreBackMidConeUpPosIndex(),
        .wrist_goal = 0.55,
        .game_piece = GamePiece::CONE_UP,
        .button = kMidBackTipConeScoreRight,
    },
    {
        .index = arm::ScoreBackMidConeDownPosIndex(),
        .wrist_goal = 2.2,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .button = kMidBackTipConeScoreRight,
    },
    {
        .index = arm::HPPickupBackConeUpIndex(),
        .wrist_goal = 0.2,
        .game_piece = GamePiece::CONE_UP,
        .button = kHPConePickup,
    },
    {
        .index = arm::ScoreFrontHighConeUpPosIndex(),
        .wrist_goal = 0.05,
        .game_piece = GamePiece::CONE_UP,
        .button = kHighBackTipConeScoreLeft,
    },
    {
        .index = arm::ScoreFrontMidConeUpPosIndex(),
        .wrist_goal = 0.05,
        .game_piece = GamePiece::CONE_UP,
        .button = kMidBackTipConeScoreLeft,
    },
};

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

  GamePiece current_game_piece_ = GamePiece::CONE_UP;

  void HandleTeleop(
      const ::frc971::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    if (!superstructure_status_fetcher_->has_wrist()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    double wrist_goal = 0.0;
    RollerGoal roller_goal = RollerGoal::IDLE;
    arm_goal_position_ = arm::NeutralPosIndex();
    std::optional<double> score_wrist_goal = std::nullopt;

    if (data.IsPressed(kGroundPickupConeUp) || data.IsPressed(kHPConePickup)) {
      roller_goal = RollerGoal::INTAKE;
      current_game_piece_ = GamePiece::CONE_UP;
    } else if (data.IsPressed(kGroundPickupConeDown)) {
      roller_goal = RollerGoal::INTAKE;
      current_game_piece_ = GamePiece::CONE_DOWN;
    }

    // Search for the active setpoint.
    for (const ArmSetpoint &setpoint : setpoints) {
      if (data.IsPressed(setpoint.button)) {
        if (setpoint.game_piece == current_game_piece_) {
          wrist_goal = setpoint.wrist_goal;
          arm_goal_position_ = setpoint.index;
          score_wrist_goal = setpoint.score_wrist_goal;
          break;
        }
      }
    }

    if (data.IsPressed(kSuck)) {
      roller_goal = RollerGoal::INTAKE;
    } else if (data.IsPressed(kSpit)) {
      if (score_wrist_goal.has_value()) {
        wrist_goal = score_wrist_goal.value();

        // If we are supposed to dunk it, wait until we are close enough to
        // spit.
        if (std::abs(score_wrist_goal.value() -
                     superstructure_status_fetcher_->wrist()->position()) <
            0.1) {
          roller_goal = RollerGoal::SPIT;
        }
      } else {
        roller_goal = RollerGoal::SPIT;
      }
    }

    {
      auto builder = superstructure_goal_sender_.MakeBuilder();

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
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
