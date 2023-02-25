#include "y2023/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/zeroing/wrap.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obay joystick_reader");

namespace y2023 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

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
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      arm_(values_),
      end_effector_(),
      wrist_(values->wrist.subsystem_params) {
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
    arm_.Reset();
    end_effector_.Reset();
    wrist_.Reset();
  }

  OutputT output_struct;
  if (joystick_state_fetcher_.Fetch() &&
      joystick_state_fetcher_->has_alliance()) {
    alliance_ = joystick_state_fetcher_->alliance();
  }
  drivetrain_status_fetcher_.Fetch();

  const uint32_t arm_goal_position =
      unsafe_goal != nullptr ? unsafe_goal->arm_goal_position() : 0u;

  flatbuffers::Offset<superstructure::ArmStatus> arm_status_offset =
      arm_.Iterate(
          timestamp, unsafe_goal != nullptr ? &(arm_goal_position) : nullptr,
          position->arm(),
          unsafe_goal != nullptr ? unsafe_goal->trajectory_override() : false,
          output != nullptr ? &output_struct.proximal_voltage : nullptr,
          output != nullptr ? &output_struct.distal_voltage : nullptr,
          output != nullptr ? &output_struct.roll_joint_voltage : nullptr,
          status->fbb());

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus> wrist_offset =
      wrist_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->wrist() : nullptr,
          position->wrist(),
          output != nullptr ? &(output_struct.wrist_voltage) : nullptr,
          status->fbb());

  EndEffectorState end_effector_state = end_effector_.RunIteration(
      timestamp,
      unsafe_goal != nullptr ? unsafe_goal->roller_goal() : RollerGoal::IDLE,
      position->end_effector_cone_beam_break(),
      position->end_effector_cube_beam_break(), &output_struct.roller_voltage);

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();
  status_builder.add_zeroed(wrist_.zeroed() && arm_.zeroed());
  status_builder.add_estopped(wrist_.estopped() || arm_.estopped());
  status_builder.add_arm(arm_status_offset);
  status_builder.add_wrist(wrist_offset);
  status_builder.add_end_effector_state(end_effector_state);

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
