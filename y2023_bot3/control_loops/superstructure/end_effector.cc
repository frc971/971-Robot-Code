#include "y2023_bot3/control_loops/superstructure/end_effector.h"

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

EndEffector::EndEffector()
    : state_(EndEffectorState::IDLE), timer_(aos::monotonic_clock::min_time) {}

void EndEffector::RunIteration(
    const ::aos::monotonic_clock::time_point timestamp, RollerGoal roller_goal,
    bool beambreak_status, double *roller_voltage, bool preloaded_with_cube) {
  *roller_voltage = 0.0;

  if (roller_goal == RollerGoal::SPIT) {
    state_ = EndEffectorState::SPITTING;
  }

  if (roller_goal == RollerGoal::SPIT_MID) {
    state_ = EndEffectorState::SPITTING_MID;
  }

  // If we started off preloaded, skip to the loaded state.
  // Make sure we weren't already there just in case.
  if (preloaded_with_cube) {
    switch (state_) {
      case EndEffectorState::IDLE:
      case EndEffectorState::INTAKING:
        state_ = EndEffectorState::LOADED;
        break;
      case EndEffectorState::LOADED:
        break;
      case EndEffectorState::SPITTING:
        break;
      case EndEffectorState::SPITTING_MID:
        break;
    }
  }

  switch (state_) {
    case EndEffectorState::IDLE:
      // If idle and intake requested, intake
      if (roller_goal == RollerGoal::INTAKE_CUBE) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      break;
    case EndEffectorState::INTAKING:
      // If intaking and beam break is not triggered, keep intaking
      if (roller_goal == RollerGoal::INTAKE_CUBE) {
        timer_ = timestamp;
      }

      if (beambreak_status) {
        // Beam has been broken, switch to loaded.
        state_ = EndEffectorState::LOADED;
        break;
      } else if (timestamp > timer_ + constants::Values::kExtraIntakingTime()) {
        // Intaking failed, waited 1 second with no beambreak
        state_ = EndEffectorState::IDLE;
        break;
      }

      *roller_voltage = kRollerCubeSuckVoltage();

      break;
    case EndEffectorState::LOADED:
      timer_ = timestamp;
      break;
    case EndEffectorState::SPITTING:
      *roller_voltage = kRollerCubeSpitVoltage();
      break;
    case EndEffectorState::SPITTING_MID:
      *roller_voltage = kRollerCubeSpitMidVoltage();
  }
}

void EndEffector::Reset() { state_ = EndEffectorState::IDLE; }

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3