#include "y2023/control_loops/superstructure/end_effector.h"

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023/vision/game_pieces_generated.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

EndEffector::EndEffector()
    : state_(EndEffectorState::IDLE),
      game_piece_(vision::Class::NONE),
      timer_(aos::monotonic_clock::min_time),
      beambreak_(false) {}

void EndEffector::RunIteration(
    const ::aos::monotonic_clock::time_point timestamp,
    RollerGoal roller_goal, double falcon_current, double cone_position,
    bool beambreak, double *roller_voltage, bool preloaded_with_cone) {
  *roller_voltage = 0.0;

  constexpr double kMinCurrent = 40.0;
  constexpr double kMaxConePosition = 0.92;

  // If we started off preloaded, skip to the loaded state.
  // Make sure we weren't already there just in case.
  if (preloaded_with_cone) {
    switch (state_) {
      case EndEffectorState::IDLE:
      case EndEffectorState::INTAKING:
        game_piece_ = vision::Class::CONE_UP;
        state_ = EndEffectorState::LOADED;
        break;
      case EndEffectorState::LOADED:
      case EndEffectorState::SPITTING:
        break;
    }
  }

  // Let them switch game pieces
  if (roller_goal == RollerGoal::INTAKE_CONE_UP) {
    game_piece_ = vision::Class::CONE_UP;
  } else if (roller_goal == RollerGoal::INTAKE_CONE_DOWN) {
    game_piece_ = vision::Class::CONE_DOWN;
  } else if (roller_goal == RollerGoal::INTAKE_CUBE) {
    game_piece_ = vision::Class::CUBE;
  }

  bool beambreak_status =
      (((game_piece_ == vision::Class::CUBE ||
         game_piece_ == vision::Class::CONE_UP) &&
        beambreak) ||
       ((game_piece_ == vision::Class::CONE_DOWN &&
         falcon_current > kMinCurrent && cone_position < kMaxConePosition)));

  // Go into spitting if we were told to, no matter where we are
  if (roller_goal == RollerGoal::SPIT && state_ != EndEffectorState::SPITTING) {
    state_ = EndEffectorState::SPITTING;

    if (!beambreak_status) {
      timer_ = timestamp;
    }
  }

  switch (state_) {
    case EndEffectorState::IDLE:
      // If idle and intake requested, intake
      if (roller_goal == RollerGoal::INTAKE_CONE_UP ||
          roller_goal == RollerGoal::INTAKE_CONE_DOWN ||
          roller_goal == RollerGoal::INTAKE_CUBE ||
          roller_goal == RollerGoal::INTAKE_LAST) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      break;
    case EndEffectorState::INTAKING:
      // If intaking and beam break is not triggered, keep intaking
      if (roller_goal == RollerGoal::INTAKE_CONE_UP ||
          roller_goal == RollerGoal::INTAKE_CONE_DOWN ||
          roller_goal == RollerGoal::INTAKE_CUBE ||
          roller_goal == RollerGoal::INTAKE_LAST) {
        timer_ = timestamp;
      }

      if (beambreak_status) {
        // Beam has been broken, switch to loaded.
        state_ = EndEffectorState::LOADED;
        break;
      } else if (timestamp > timer_ + constants::Values::kExtraIntakingTime()) {
        // Intaking failed, waited 2 seconds with no beambreak
        state_ = EndEffectorState::IDLE;
        break;
      }

      if (game_piece_ == vision::Class::CUBE) {
        *roller_voltage = kRollerCubeSuckVoltage();
      } else {
        *roller_voltage = kRollerConeSuckVoltage();
      }

      break;
    case EndEffectorState::LOADED:
      timer_ = timestamp;
      // If loaded and beam break not triggered and not preloaded, intake
      if (!beambreak_status && !preloaded_with_cone) {
        state_ = EndEffectorState::INTAKING;
      }
      if (game_piece_ != vision::Class::CUBE) {
        *roller_voltage = 1.3;
      }
      break;
    case EndEffectorState::SPITTING:
      // If spit requested, spit
      if (game_piece_ == vision::Class::CUBE) {
        *roller_voltage = kRollerCubeSpitVoltage();
      } else {
        *roller_voltage = kRollerConeSpitVoltage();
      }
      if (beambreak_) {
        if (!beambreak_status) {
          timer_ = timestamp;
        }
      } else if (timestamp > timer_ + constants::Values::kExtraSpittingTime()) {
        // Finished spitting
        state_ = EndEffectorState::IDLE;
        game_piece_ = vision::Class::NONE;
      } else if (roller_goal == RollerGoal::INTAKE_CONE_UP ||
                 roller_goal == RollerGoal::INTAKE_CONE_DOWN ||
                 roller_goal == RollerGoal::INTAKE_CUBE ||
                 roller_goal == RollerGoal::INTAKE_LAST) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }

      break;
  }

  beambreak_ = beambreak_status;
}

void EndEffector::Reset() { state_ = EndEffectorState::IDLE; }

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
