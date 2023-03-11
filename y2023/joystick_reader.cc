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
using y2023::control_loops::drivetrain::GridSelectionHint;
using y2023::control_loops::drivetrain::RowSelectionHint;
using y2023::control_loops::drivetrain::SpotSelectionHint;
using y2023::control_loops::drivetrain::TargetSelectorHint;
using y2023::control_loops::superstructure::RollerGoal;
using Side = frc971::control_loops::drivetrain::RobotSide;

namespace y2023 {
namespace input {
namespace joysticks {

constexpr double kConeWrist = 0.4;
constexpr double kCubeWrist = 1.0;

// TODO(milind): add correct locations
const ButtonLocation kDriverSpit(2, 1);
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
const ButtonLocation kGroundPickupConeDown(4, 8);
const ButtonLocation kGroundPickupCube(4, 10);
const ButtonLocation kHPConePickup(4, 6);

const ButtonLocation kSuck(4, 11);
const ButtonLocation kBack(4, 12);

const ButtonLocation kWrist(4, 10);
const ButtonLocation kStayIn(3, 4);

const ButtonLocation kConeDownTip(4, 4);
const ButtonLocation kConeDownBase(4, 5);

namespace superstructure = y2023::control_loops::superstructure;
namespace arm = superstructure::arm;

enum class GamePiece {
  CONE_UP = 0,
  CONE_DOWN = 1,
  CUBE = 2,
  CONE_TIP = 4,
};

struct ButtonData {
  ButtonLocation button;
  std::optional<SpotSelectionHint> spot = std::nullopt;
};

struct ArmSetpoint {
  uint32_t index;
  std::optional<uint32_t> place_index = std::nullopt;
  double wrist_goal;
  std::optional<double> score_wrist_goal = std::nullopt;
  GamePiece game_piece;
  std::vector<ButtonData> buttons;
  Side side;
  std::optional<RowSelectionHint> row_hint = std::nullopt;
};

const std::vector<ArmSetpoint> setpoints = {
    {
        .index = arm::GroundPickupBackConeUpIndex(),
        .wrist_goal = 0.7,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kGroundPickupConeUp}},
        .side = Side::BACK,
    },
    {
        .index = arm::GroundPickupFrontConeUpIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kGroundPickupConeUp}},
        .side = Side::FRONT,
    },
    {
        .index = arm::GroundPickupBackConeDownBaseIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kGroundPickupConeDown}},
        .side = Side::BACK,
    },
    {
        .index = arm::GroundPickupFrontConeDownBaseIndex(),
        .wrist_goal = 0.6,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kGroundPickupConeDown}},
        .side = Side::FRONT,
    },
    {
        .index = arm::ScoreBackLowConeDownTipIndex(),
        .wrist_goal = 0.7,
        .game_piece = GamePiece::CONE_TIP,
        .buttons = {{kLowConeScoreRight, SpotSelectionHint::RIGHT},
                    {kLowCube, SpotSelectionHint::MIDDLE},
                    {kLowConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreBackMidConeDownTipIndex(),
        .place_index = arm::ScoreBackMidConeDownTipPlacedIndex(),
        .wrist_goal = 0.8,
        .score_wrist_goal = 2.0,
        .game_piece = GamePiece::CONE_TIP,
        .buttons = {{kMidConeScoreRight, SpotSelectionHint::RIGHT},
                    {kMidConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontMidConeDownTipIndex(),
        .place_index = arm::ScoreFrontMidConeDownTipPlacedIndex(),
        .wrist_goal = 0.0,
        .score_wrist_goal = 1.4,
        .game_piece = GamePiece::CONE_TIP,
        .buttons = {{kMidConeScoreRight, SpotSelectionHint::RIGHT},
                    {kMidConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontHighConeDownTipIndex(),
        .place_index = arm::ScoreFrontHighConeDownTipPlacedIndex(),
        .wrist_goal = 0.4,
        .score_wrist_goal = 1.4,
        .game_piece = GamePiece::CONE_TIP,
        .buttons = {{kHighConeScoreRight, SpotSelectionHint::RIGHT},
                    {kHighConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::ScoreFrontLowConeDownTipIndex(),
        .wrist_goal = 2.8,
        .game_piece = GamePiece::CONE_TIP,
        .buttons = {{kLowConeScoreRight, SpotSelectionHint::RIGHT},
                    {kLowCube, SpotSelectionHint::MIDDLE},
                    {kLowConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::ScoreBackMidConeUpIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kMidConeScoreRight, SpotSelectionHint::RIGHT},
                    {kMidConeScoreLeft, SpotSelectionHint::LEFT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackLowConeUpIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kLowConeScoreLeft, SpotSelectionHint::LEFT},
                    {kLowConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreFrontLowConeUpIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kLowConeScoreLeft, SpotSelectionHint::LEFT},
                    {kLowConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreBackMidConeDownBaseIndex(),
        .wrist_goal = 2.5,
        .score_wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kMidConeScoreLeft, SpotSelectionHint::LEFT},
                    {kMidConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackLowConeDownBaseIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kLowConeScoreLeft, SpotSelectionHint::LEFT},
                    {kLowConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreFrontLowConeDownBaseIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kLowConeScoreLeft, SpotSelectionHint::LEFT},
                    {kLowConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreFrontMidConeDownBaseIndex(),
        .wrist_goal = 2.6,
        .score_wrist_goal = 0.2,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kMidConeScoreLeft, SpotSelectionHint::LEFT},
                    {kMidConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontHighConeDownBaseIndex(),
        .wrist_goal = 2.6,
        .score_wrist_goal = 0.2,
        .game_piece = GamePiece::CONE_DOWN,
        .buttons = {{kHighConeScoreLeft, SpotSelectionHint::LEFT},
                    {kHighConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::HPPickupFrontConeUpIndex(),
        .wrist_goal = kConeWrist,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kHPConePickup}},
        .side = Side::FRONT,
    },
    {
        .index = arm::HPPickupBackConeUpIndex(),
        .wrist_goal = 0.5,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kHPConePickup}},
        .side = Side::BACK,
    },
    {
        .index = arm::ScoreFrontHighConeUpIndex(),
        .wrist_goal = kConeWrist + 0.05,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kHighConeScoreLeft, SpotSelectionHint::LEFT},
                    {kHighConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::ScoreFrontMidConeUpIndex(),
        .wrist_goal = kConeWrist + 0.05,
        .game_piece = GamePiece::CONE_UP,
        .buttons = {{kMidConeScoreLeft, SpotSelectionHint::LEFT},
                    {kMidConeScoreRight, SpotSelectionHint::RIGHT}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::GroundPickupBackCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kGroundPickupCube}},
        .side = Side::BACK,
    },
    {
        .index = arm::ScoreFrontMidCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kMidCube, SpotSelectionHint::MIDDLE}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreBackMidCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kMidCube, SpotSelectionHint::MIDDLE}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::MIDDLE,
    },
    {
        .index = arm::ScoreFrontLowCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kLowCube, SpotSelectionHint::MIDDLE}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreBackLowCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kLowCube, SpotSelectionHint::MIDDLE}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::BOTTOM,
    },
    {
        .index = arm::ScoreFrontHighCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kHighCube, SpotSelectionHint::MIDDLE}},
        .side = Side::FRONT,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::ScoreBackHighCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kHighCube, SpotSelectionHint::MIDDLE}},
        .side = Side::BACK,
        .row_hint = RowSelectionHint::TOP,
    },
    {
        .index = arm::GroundPickupFrontCubeIndex(),
        .wrist_goal = kCubeWrist,
        .game_piece = GamePiece::CUBE,
        .buttons = {{kGroundPickupCube}},
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
    std::optional<double> place_index = std::nullopt;

    if (data.IsPressed(kGroundPickupConeUp) || data.IsPressed(kHPConePickup)) {
      roller_goal = RollerGoal::INTAKE_CONE_UP;
      current_game_piece_ = GamePiece::CONE_UP;
    } else if (data.IsPressed(kGroundPickupConeDown)) {
      roller_goal = RollerGoal::INTAKE_CONE_DOWN;
      current_game_piece_ = GamePiece::CONE_DOWN;
    } else if (data.IsPressed(kGroundPickupCube)) {
      roller_goal = RollerGoal::INTAKE_CUBE;
      current_game_piece_ = GamePiece::CUBE;
    }

    if (current_game_piece_ == GamePiece::CONE_DOWN ||
        current_game_piece_ == GamePiece::CONE_TIP) {
      if (data.IsPressed(kConeDownTip)) {
        current_game_piece_ = GamePiece::CONE_TIP;
      } else if (data.IsPressed(kConeDownBase)) {
        current_game_piece_ = GamePiece::CONE_DOWN;
      }
    }

    if (current_game_piece_ == GamePiece::CUBE) {
      wrist_goal = kCubeWrist;
    }

    std::optional<RowSelectionHint> placing_row;
    std::optional<SpotSelectionHint> placing_spot;

    // Keep the setpoint if the button is still held.  This lets us release the
    // back button once a side has been selected.
    if (current_setpoint_ != nullptr) {
      bool found = false;
      for (const auto &button : current_setpoint_->buttons) {
        if (data.IsPressed(button.button)) {
          found = true;
          placing_spot = button.spot;
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
        for (const auto &button : setpoint.buttons) {
          if (data.IsPressed(button.button)) {
            if (setpoint.game_piece == current_game_piece_ &&
                setpoint.side == current_side) {
              current_setpoint_ = &setpoint;
              placing_spot = button.spot;
            }
          }
        }
      }
    }

    // And, pull the bits out of it.
    if (current_setpoint_ != nullptr) {
      if (!data.IsPressed(kStayIn)) {
        wrist_goal = current_setpoint_->wrist_goal;
        arm_goal_position_ = current_setpoint_->index;
        score_wrist_goal = current_setpoint_->score_wrist_goal;
        place_index = current_setpoint_->place_index;
      }

      placing_row = current_setpoint_->row_hint;
    }

    CHECK_EQ(placing_row.has_value(), placing_spot.has_value());

    if (data.IsPressed(kSuck)) {
      roller_goal = RollerGoal::INTAKE_LAST;
    } else if (data.IsPressed(kSpit) || data.IsPressed(kDriverSpit)) {
      if (score_wrist_goal.has_value()) {
        wrist_goal = score_wrist_goal.value();

        // If we are supposed to dunk it, wait until we are close enough to
        // spit.
        if (std::abs(score_wrist_goal.value() -
                     superstructure_status_fetcher_->wrist()->goal_position()) <
            0.1) {
          if (place_index.has_value()) {
            arm_goal_position_ = place_index.value();
            if (arm_goal_position_ ==
                    superstructure_status_fetcher_->arm()->current_node() &&
                superstructure_status_fetcher_->arm()->path_distance_to_go() <
                    0.01) {
              roller_goal = RollerGoal::SPIT;
            }
          } else {
            roller_goal = RollerGoal::SPIT;
          }
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
      hint_builder.add_robot_side(CHECK_NOTNULL(current_setpoint_)->side);
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
