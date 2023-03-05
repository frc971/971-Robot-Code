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
#include "y2023/control_loops/drivetrain/target_selector_hint_generated.h"
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
using y2023::control_loops::drivetrain::RowSelectionHint;
using y2023::control_loops::drivetrain::GridSelectionHint;
using y2023::control_loops::drivetrain::SpotSelectionHint;
using y2023::control_loops::drivetrain::TargetSelectorHint;

namespace y2023 {
namespace input {
namespace joysticks {

// TODO(milind): add correct locations
const ButtonLocation kScore(4, 4);
const ButtonLocation kSpit(4, 13);

const ButtonLocation kHighConeScoreLeft(4, 14);
const ButtonLocation kHighConeScoreRight(3, 1);

const ButtonLocation kMidConeScoreLeft(4, 15);
const ButtonLocation kMidConeScoreRight(3, 2);

const ButtonLocation kLowConeScoreLeft(4, 16);
const ButtonLocation kLowConeScoreRight(3, 3);

const ButtonLocation kHighCube(4, 1);
const ButtonLocation kMidCube(4, 2);
const ButtonLocation kLowCube(4, 3);

const ButtonLocation kGroundPickupConeUp(4, 7);
const ButtonLocation kGroundPickupConeDownBase(4, 8);
const ButtonLocation kGroundPickupCube(4, 10);
const ButtonLocation kHPConePickup(4, 6);

const ButtonLocation kSuck(4, 11);
const ButtonLocation kBack(4, 12);

const ButtonLocation kWrist(4, 10);

namespace superstructure = y2023::control_loops::superstructure;
namespace arm = superstructure::arm;

enum class GamePiece {
  CONE_UP = 0,
  CONE_DOWN = 1,
  CUBE = 2,
};

enum class Side {
  FRONT = 0,
  BACK = 1,
};

struct ArmSetpoint {
  uint32_t index;
  double wrist_goal;
  std::optional<double> score_wrist_goal = std::nullopt;
  GamePiece game_piece;
  std::vector<ButtonLocation> buttons;
  Side side;
  std::optional<RowSelectionHint> row_hint = std::nullopt;
  std::optional<SpotSelectionHint> spot_hint = std::nullopt;
};

const std::vector<ArmSetpoint> setpoints = {
    {
        .index = arm::GroundPickupBackConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kGroundPickupConeUp},
        .side = Side::BACK,
    },
    {
        .index = arm::GroundPickupFrontConeUpIndex(),
        .wrist_goal = 0.2,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kGroundPickupConeUp},
        .side = Side::FRONT,
    },
    {
        .index = arm::GroundPickupBackConeDownBaseIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kGroundPickupConeDownBase},
        .side = Side::BACK,
    },
    {
        .index = arm::GroundPickupFrontConeDownBaseIndex(),
        .wrist_goal = 0.2,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kGroundPickupConeDownBase},
        .side = Side::FRONT,
    },
    {
        .index = arm::ScoreBackMidConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kMidConeScoreRight, kMidConeScoreLeft},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::RIGHT,
    },
    {
        .index = arm::ScoreBackLowConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kLowConeScoreLeft, kLowConeScoreRight},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::ScoreFrontLowConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kLowConeScoreLeft, kLowConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::ScoreBackMidConeDownBaseIndex(),
        .wrist_goal = 2.2,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kMidConeScoreLeft, kMidConeScoreRight},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::RIGHT,
    },
    {
        .index = arm::ScoreBackLowConeDownBaseIndex(),
        .wrist_goal = 0.0,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kLowConeScoreLeft, kLowConeScoreRight},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::RIGHT,
    },
    {
        .index = arm::ScoreFrontLowConeDownBaseIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kLowConeScoreLeft, kLowConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::ScoreFrontMidConeDownBaseIndex(),
        .wrist_goal = 2.0,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kMidConeScoreLeft, kMidConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::ScoreFrontHighConeDownBaseIndex(),
        .wrist_goal = 2.0,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {kHighConeScoreLeft, kHighConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::HPPickupFrontConeUpIndex(),
        .wrist_goal = 0.0,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kHPConePickup},
        .side = Side::FRONT,
    },
    {
        .index = arm::HPPickupBackConeUpIndex(),
        .wrist_goal = 0.4,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kHPConePickup},
        .side = Side::BACK,
    },
    {
        .index = arm::ScoreFrontHighConeUpIndex(),
        .wrist_goal = 0.05,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kHighConeScoreLeft, kHighConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::ScoreFrontMidConeUpIndex(),
        .wrist_goal = 0.05,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {kMidConeScoreLeft, kMidConeScoreRight},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::LEFT,
    },
    {
        .index = arm::GroundPickupBackCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kGroundPickupCube},
        .side = Side::BACK,
    },
    {
        .index = arm::ScoreFrontMidCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kMidCube},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackMidCubeIndex(),
        .wrist_goal = 0.6,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CUBE,
        .buttons = {kMidCube},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontLowCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kLowCube},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackLowCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kLowCube},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontHighCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kHighCube},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackHighCubeIndex(),
        .wrist_goal = 0.6,
        .score_wrist_goal = 0.0,
        .game_piece = GamePiece::CUBE,
        .buttons = {kHighCube},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::TOP,
        .spot_hint = SpotSelectionHint::MIDDLE,
    },
    {
        .index = arm::GroundPickupFrontCubeIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CUBE,
        .buttons = {kGroundPickupCube},
        .side = Side::FRONT,
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
        target_selector_hint_sender_(
            event_loop->MakeSender<TargetSelectorHint>("/drivetrain")),
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
    arm_goal_position_ = arm::NeutralIndex();
    std::optional<double> score_wrist_goal = std::nullopt;

    if (data.IsPressed(kGroundPickupConeUp) || data.IsPressed(kHPConePickup)) {
      roller_goal = RollerGoal::INTAKE_CONE;
      current_game_piece_ = GamePiece::CONE_UP;
    } else if (data.IsPressed(kGroundPickupConeDownBase)) {
      roller_goal = RollerGoal::INTAKE_CONE;
      current_game_piece_ = GamePiece::CONE_DOWN;
    } else if (data.IsPressed(kGroundPickupCube)) {
      roller_goal = RollerGoal::INTAKE_CUBE;
      current_game_piece_ = GamePiece::CUBE;
    }

    if (current_game_piece_ == GamePiece::CUBE) {
      wrist_goal = 0.6;
    }

    std::optional<RowSelectionHint> placing_row;
    std::optional<SpotSelectionHint> placing_spot;

    // Keep the setpoint if the button is still held.  This lets us release the
    // back button once a side has been selected.
    if (current_setpoint_ != nullptr) {
      bool found = false;
      for (const ButtonLocation &button : current_setpoint_->buttons) {
        if (data.IsPressed(button)) {
          found = true;
        }
      }
      if (!found) {
        current_setpoint_ = nullptr;
      }
    }

    // Ok, no active setpoint.  Search for the right one.
    if (current_setpoint_ == nullptr) {
      const Side current_side =
          data.IsPressed(kBack) ? Side::BACK : Side::FRONT;
      // Search for the active setpoint.
      for (const ArmSetpoint &setpoint : setpoints) {
        for (const ButtonLocation &button : setpoint.buttons) {
          if (data.IsPressed(button)) {
            if (setpoint.game_piece == current_game_piece_ &&
                setpoint.side == current_side) {
              current_setpoint_ = &setpoint;
            }
          }
        }
      }
    }

    // And, pull the bits out of it.
    if (current_setpoint_ != nullptr) {
      wrist_goal = current_setpoint_->wrist_goal;
      arm_goal_position_ = current_setpoint_->index;
      score_wrist_goal = current_setpoint_->score_wrist_goal;
      placing_row = current_setpoint_->row_hint;
      placing_spot = current_setpoint_->spot_hint;
    }

    CHECK_EQ(placing_row.has_value(), placing_spot.has_value());

    if (data.IsPressed(kSuck)) {
      roller_goal = RollerGoal::INTAKE_LAST;
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
    if (placing_row.has_value()) {
      auto builder = target_selector_hint_sender_.MakeBuilder();
      auto hint_builder = builder.MakeBuilder<TargetSelectorHint>();
      hint_builder.add_row(placing_row.value());
      hint_builder.add_spot(placing_spot.value());
      // TODO: Add field to TargetSelector hint for forwards vs. backwards
      // placement.
      if (builder.Send(hint_builder.Finish()) != aos::RawSender::Error::kOk) {
        AOS_LOG(ERROR, "Sending target selector hint failed.\n");
      }
    }
  }

 private:
  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;
  ::aos::Sender<TargetSelectorHint> target_selector_hint_sender_;

  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;

  uint32_t arm_goal_position_;

  const ArmSetpoint *current_setpoint_ = nullptr;
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
