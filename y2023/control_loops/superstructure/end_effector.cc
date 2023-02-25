#include "y2023/control_loops/superstructure/end_effector.h"

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

EndEffector::EndEffector()
    : state_(EndEffectorState::IDLE), beambreak_(false) {}

EndEffectorState EndEffector::RunIteration(
    const ::aos::monotonic_clock::time_point timestamp, RollerGoal roller_goal,
    double falcon_current, double cone_position, bool beambreak,
    double *roller_voltage) {
  *roller_voltage = 0.0;

  constexpr double kMinCurrent = 20.0;
  constexpr double kMaxConePosition = 0.92;

  bool beambreak_status = (beambreak || (falcon_current > kMinCurrent &&
                                         cone_position < kMaxConePosition));

  switch (state_) {
    case EndEffectorState::IDLE:
      // If idle and intake requested, intake
      if (roller_goal == RollerGoal::INTAKE) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      break;
    case EndEffectorState::INTAKING:
      // If intaking and beam break is not triggered, keep intaking
      if (beambreak_status) {
        // Beam has been broken, switch to loaded.
        state_ = EndEffectorState::LOADED;
        break;
      } else if (timestamp > timer_ + constants::Values::kExtraIntakingTime()) {
        // Intaking failed, waited 2 seconds with no beambreak
        state_ = EndEffectorState::IDLE;
        break;
      }

      *roller_voltage = constants::Values::kRollerVoltage();

      break;
    case EndEffectorState::LOADED:
      // If loaded and beam break not triggered, intake
      if (!beambreak_status) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      // If loaded and spit requested, spit
      else if (roller_goal == RollerGoal::SPIT) {
        state_ = EndEffectorState::SPITTING;
      }
      break;
    case EndEffectorState::SPITTING:
      // If spit requested, spit
      *roller_voltage = -constants::Values::kRollerVoltage();
      if (beambreak_) {
        if (!beambreak_status) {
          timer_ = timestamp;
        }
      } else if (timestamp > timer_ + constants::Values::kExtraSpittingTime()) {
        // Finished spitting
        state_ = EndEffectorState::IDLE;
      }

      break;
  }

  beambreak_ = beambreak_status;

  return state_;
}

void EndEffector::Reset() { state_ = EndEffectorState::IDLE; }

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
