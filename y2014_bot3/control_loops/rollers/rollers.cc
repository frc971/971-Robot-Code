#include "bot3/control_loops/rollers/rollers.h"
#include "bot3/control_loops/rollers/rollers.q.h"

namespace bot3 {
namespace control_loops {

void RollersLoop::RunIteration(const Rollers::Goal *goal,
                               const Rollers::Position * /*position*/,
                               Rollers::Output *output,
                               Rollers::Status * /*status*/) {
  constexpr double kBot3IntakeForwardVoltage = 12.0;
  constexpr double kBot3IntakeBackwardVoltage = -12.0;
  constexpr double kBot3LowGoalForwardVoltage = 6.0;
  constexpr double kBot3LowGoalBackwardVoltage = -6.0;

  const int intake = goal->intake;
  const int low_spit = goal->low_spit;
  const bool human_player = goal->human_player;

  if (!output) {
    return;
  }

  output->Zero();

  switch (low_spit) {
    case 1:
      // Spit towards front
      output->low_goal_voltage = kBot3LowGoalBackwardVoltage;
      output->front_intake_voltage = kBot3IntakeBackwardVoltage;
      output->back_intake_voltage = -kBot3IntakeForwardVoltage;
      break;
    case -1:
      // Spit towards back
      output->low_goal_voltage = kBot3LowGoalForwardVoltage;
      output->back_intake_voltage = -kBot3IntakeBackwardVoltage;
      output->front_intake_voltage = kBot3IntakeForwardVoltage;
      break;
    default:
      // Stationary
      break;
  }

  switch (intake) {
    case 1:
      // Front intake.
      output->front_extended = true;
      output->back_extended = false;
      output->front_intake_voltage = kBot3IntakeForwardVoltage;
      output->back_intake_voltage = 0.0;
      break;
    case -1:
      // Back intake.
      output->back_extended = true;
      output->front_extended = false;
      output->back_intake_voltage = -kBot3IntakeForwardVoltage;
      output->front_intake_voltage = 0.0;
      break;
    default:
      // Stationary
      break;
  }

  if (human_player) {
    // Intake for human player.
    output->front_extended = false;
    output->back_extended = false;
    output->front_intake_voltage = kBot3IntakeForwardVoltage;
    output->back_intake_voltage = -kBot3IntakeForwardVoltage;
  }
}

}  //  namespace control_loops
}  //  namespace bot3
