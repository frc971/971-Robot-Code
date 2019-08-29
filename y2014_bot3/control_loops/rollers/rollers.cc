#include "y2014_bot3/control_loops/rollers/rollers.h"

#include "aos/logging/logging.h"
#include "y2014_bot3/control_loops/rollers/rollers_goal_generated.h"
#include "y2014_bot3/control_loops/rollers/rollers_output_generated.h"
#include "y2014_bot3/control_loops/rollers/rollers_position_generated.h"
#include "y2014_bot3/control_loops/rollers/rollers_status_generated.h"

namespace y2014_bot3 {
namespace control_loops {
namespace rollers {

Rollers::Rollers(::aos::EventLoop *event_loop, const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name) {}

void Rollers::RunIteration(const Goal *goal, const Position * /*position*/,
                           aos::Sender<Output>::Builder *output,
                           aos::Sender<Status>::Builder *status) {
  constexpr double k2014Bot3IntakeForwardVoltage = 12.0;
  constexpr double k2014Bot3IntakeBackwardVoltage = -12.0;
  constexpr double k2014Bot3LowGoalForwardVoltage = 6.0;
  constexpr double k2014Bot3LowGoalBackwardVoltage = -6.0;

  status->Send(status->MakeBuilder<Status>().Finish());

  if (!output || !goal) {
    return;
  }

  const int intake = goal->intake();
  const int low_spit = goal->low_spit();
  const bool human_player = goal->human_player();

  OutputT output_struct;

  switch (low_spit) {
    case 1:
      // Spit towards front
      output_struct.low_goal_voltage = k2014Bot3LowGoalBackwardVoltage;
      output_struct.front_intake_voltage = k2014Bot3IntakeBackwardVoltage;
      output_struct.back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
      break;
    case -1:
      // Spit towards back
      output_struct.low_goal_voltage = k2014Bot3LowGoalForwardVoltage;
      output_struct.back_intake_voltage = -k2014Bot3IntakeBackwardVoltage;
      output_struct.front_intake_voltage = k2014Bot3IntakeForwardVoltage;
      break;
    default:
      // Stationary
      break;
  }

  switch (intake) {
    case 1:
      // Front intake.
      output_struct.front_extended = true;
      output_struct.back_extended = false;
      output_struct.front_intake_voltage = k2014Bot3IntakeForwardVoltage;
      output_struct.back_intake_voltage = 0.0;
      break;
    case -1:
      // Back intake.
      output_struct.back_extended = true;
      output_struct.front_extended = false;
      output_struct.back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
      output_struct.front_intake_voltage = 0.0;
      break;
    default:
      // Stationary
      break;
  }

  if (human_player) {
    // Intake for human player.
    output_struct.front_extended = false;
    output_struct.back_extended = false;
    output_struct.front_intake_voltage = k2014Bot3IntakeForwardVoltage;
    output_struct.back_intake_voltage = -k2014Bot3IntakeForwardVoltage;
  }

  output->Send(Output::Pack(*output->fbb(), &output_struct));
}

}  //  namespace rollers
}  //  namespace control_loops
}  //  namespace y2014_bot3
