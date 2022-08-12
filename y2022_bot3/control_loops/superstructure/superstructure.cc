#include "y2022_bot3/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "frc971/zeroing/wrap.h"

namespace y2022_bot3 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               std::shared_ptr<const constants::Values> values,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")) {
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

  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  drivetrain_status_fetcher_.Fetch();

  output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_zeroed(true);
  status_builder.add_estopped(false);

  (void)status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022_bot3
