#include "y2023_bot3/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/wrap.h"

ABSL_FLAG(bool, ignore_distance, false,
          "If true, ignore distance when shooting and obay joystick_reader");

namespace y2023_bot3::control_loops::superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               std::shared_ptr<const constants::Values> values,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  (void)unsafe_goal;
  (void)position;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
  }

  OutputT output_struct;

  Status::Builder status_builder = status->MakeBuilder<Status>();

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  status_builder.add_zeroed(true);

  (void)status->Send(status_builder.Finish());
}

}  // namespace y2023_bot3::control_loops::superstructure
