#include "y2022/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops:: RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      climber_(constants::GetValues().climber.subsystem_params) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
  }

  OutputT output_struct;

  flatbuffers::Offset<RelativeEncoderProfiledJointStatus>
      climber_status_offset = climber_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->climber() : nullptr,
          position->climber(),
          output != nullptr ? &(output_struct.climber_voltage) : nullptr,
          status->fbb());

  if (output != nullptr) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  // Climber is always zeroed; only has a pot
  status_builder.add_zeroed(climber_.zeroed());
  status_builder.add_estopped(climber_.estopped());
  status_builder.add_climber(climber_status_offset);

  (void)status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
