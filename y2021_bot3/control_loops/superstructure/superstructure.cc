#include "y2021_bot3/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"

namespace y2021_bot3 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position * /*position*/,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
  }

  if (output != nullptr && unsafe_goal != nullptr) {
    OutputT output_struct;

    output_struct.intake_volts =
        std::clamp(unsafe_goal->intake_speed(), -12.0, 12.0);
    output_struct.outtake_volts =
        std::clamp(unsafe_goal->outtake_speed(), -12.0, 12.0);
    output_struct.climber_volts =
        std::clamp(unsafe_goal->climber_speed(), -12.0, 12.0);
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_zeroed(true);
  status_builder.add_estopped(false);

  if (unsafe_goal != nullptr) {
    status_builder.add_intake_speed(unsafe_goal->intake_speed());
    status_builder.add_outtake_speed(unsafe_goal->outtake_speed());
    status_builder.add_climber_speed(unsafe_goal->climber_speed());
  }

  (void)status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2021_bot3
