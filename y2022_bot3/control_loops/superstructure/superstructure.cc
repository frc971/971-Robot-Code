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
      climber_left_(values_->climber_left.subsystem_params),
      climber_right_(values_->climber_right.subsystem_params),
      intake_(values_->intake.subsystem_params),
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
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
  }

  OutputT output_struct;

  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }

  drivetrain_status_fetcher_.Fetch();

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      intake_status_offset = intake_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake() : nullptr,
          position->intake(),
          output != nullptr ? &output_struct.intake_voltage : nullptr,
          status->fbb());

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      climber_left_status_offset = climber_left_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->climber_left() : nullptr,
          position->climber_left(),
          output != nullptr ? &output_struct.climber_voltage_left : nullptr,
          status->fbb());

  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      climber_right_status_offset = climber_right_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->climber_right() : nullptr,
          position->climber_right(),
          output != nullptr ? &output_struct.climber_voltage_right : nullptr,
          status->fbb());

  if (output != nullptr) {
    output_struct.roller_voltage =
        (unsafe_goal != nullptr ? unsafe_goal->roller_speed() : 0.0);
  }
  output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed =
      intake_.zeroed() && climber_left_.zeroed() && climber_right_.zeroed();
  const bool estopped = intake_.estopped() || climber_left_.estopped() ||
                        climber_right_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_intake(intake_status_offset);
  status_builder.add_climber_left(climber_left_status_offset);
  status_builder.add_climber_right(climber_right_status_offset);

  (void)status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022_bot3
