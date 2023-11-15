#include "y2023_bot3/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/wrap.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obay joystick_reader");

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               std::shared_ptr<const constants::Values> values,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values),
      end_effector_(),
      pivot_joint_(values) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  const monotonic_clock::time_point timestamp =
      event_loop()->context().monotonic_event_time;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    end_effector_.Reset();
  }

  OutputT output_struct;

  end_effector_.RunIteration(
      timestamp,
      unsafe_goal != nullptr ? unsafe_goal->roller_goal() : RollerGoal::IDLE,
      position->end_effector_cube_beam_break(), &output_struct.roller_voltage,
      unsafe_goal != nullptr ? unsafe_goal->preloaded_with_cube() : false);

  flatbuffers::Offset<
      frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
      pivot_joint_offset = pivot_joint_.RunIteration(
          unsafe_goal != nullptr ? unsafe_goal->pivot_goal()
                                 : PivotGoal::NEUTRAL,
          output != nullptr ? &(output_struct.pivot_joint_voltage) : nullptr,
          position->pivot_joint_position(), status->fbb());

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_zeroed(pivot_joint_.zeroed());
  status_builder.add_estopped(pivot_joint_.estopped());
  status_builder.add_pivot_joint(pivot_joint_offset);
  status_builder.add_end_effector_state(end_effector_.state());

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  (void)status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3
