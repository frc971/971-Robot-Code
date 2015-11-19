#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/common/logging/logging.h"

namespace y2014_bot3 {
namespace control_loops {

Rollers::Rollers(control_loops::RollersQueue *rollers)
    : aos::controls::ControlLoop<control_loops::RollersQueue>(rollers) {}

void Rollers::RunIteration(
    const control_loops::RollersQueue::Goal *goal,
    const control_loops::RollersQueue::Position * /*position*/,
    control_loops::RollersQueue::Output *output,
    control_loops::RollersQueue::Status * /*status*/) {
  constexpr double k2014Bot3IntakeForwardVoltage = 12.0;
  constexpr double k2014Bot3IntakeBackwardVoltage = -12.0;
  constexpr double k2014Bot3LowGoalForwardVoltage = 6.0;
  constexpr double k2014Bot3LowGoalBackwardVoltage = -6.0;

  if (!output || !goal) {
    return;
  }

  const int intake = goal->intake;
  const int low_spit = goal->low_spit;
  const bool human_player = goal->human_player;

  output->Zero();

  switch (low_spit) {
    case 1:
      // Spit towards front
      output->low_goal_voltage = k2014Bot3LowGoalBackwardVoltage;
      output->front_intake_voltage = k2014Bot3IntakeBackwardVoltage;
      output->back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
      break;
    case -1:
      // Spit towards back
      output->low_goal_voltage = k2014Bot3LowGoalForwardVoltage;
      output->back_intake_voltage = -k2014Bot3IntakeBackwardVoltage;
      output->front_intake_voltage = k2014Bot3IntakeForwardVoltage;
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
      output->front_intake_voltage = k2014Bot3IntakeForwardVoltage;
      output->back_intake_voltage = 0.0;
      break;
    case -1:
      // Back intake.
      output->back_extended = true;
      output->front_extended = false;
      output->back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
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
    output->front_intake_voltage = k2014Bot3IntakeForwardVoltage;
    output->back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
  }
}

}  //  namespace control_loops
}  //  namespace y2014_bot3
