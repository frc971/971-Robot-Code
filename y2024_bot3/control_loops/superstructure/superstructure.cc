#include "y2024_bot3/control_loops/superstructure/superstructure.h"

#include <chrono>

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "frc971/zeroing/wrap.h"

ABSL_FLAG(bool, ignore_distance, false,
          "If true, ignore distance when shooting and obey joystick_reader");

namespace y2024_bot3::control_loops::superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      constants_fetcher_(event_loop),
      robot_constants_(&constants_fetcher_.constants()),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")) {
  event_loop->SetRuntimeRealtimePriority(30);
}

bool PositionNear(double position, double goal, double threshold) {
  return std::abs(position - goal) < threshold;
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  (void)position;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
  }

  OutputT output_struct;

  if (unsafe_goal != nullptr) {
  }

  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = true;
  const bool estopped = false;

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);

  (void)status->Send(status_builder.Finish());
}
}  // namespace y2024_bot3::control_loops::superstructure
