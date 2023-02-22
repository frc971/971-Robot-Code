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
    const ::aos::monotonic_clock::time_point timestamp, bool intake, bool spit,
    bool cone_beambreak, bool cube_beambreak, double *roller_voltage) {
  bool beambreak = cone_beambreak || cube_beambreak;

  *roller_voltage = 0.0;

  switch (state_) {
    case EndEffectorState::IDLE:
      // If idle and intake requested, intake
      if (intake) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      break;
    case EndEffectorState::INTAKING:
      // If intaking and beam break is not triggered, keep intaking
      if (beambreak) {
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
      if (!beambreak) {
        state_ = EndEffectorState::INTAKING;
        timer_ = timestamp;
      }
      // If loaded and spit requested, spit
      else if (spit) {
        state_ = EndEffectorState::SPITTING;
      }
      break;

    case EndEffectorState::SPITTING:
      // If spit requested, spit
      *roller_voltage = -constants::Values::kRollerVoltage();
      if (beambreak_) {
        if (!beambreak) {
          timer_ = timestamp;
        }
      } else if (timestamp > timer_ + constants::Values::kExtraSpittingTime()) {
        // Finished spitting
        state_ = EndEffectorState::IDLE;
      }

      break;
  }

  beambreak_ = beambreak;

  return state_;
}

EndEffectorState EndEffector::state() { return state_; }

void EndEffector::Reset() { state_ = EndEffectorState::IDLE; }

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
