#include "y2023/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/network/team_number.h"
#include "frc971/shooter_interpolation/interpolation.h"
#include "frc971/zeroing/wrap.h"
#include "y2023/control_loops/superstructure/arm/arm_trajectories_generated.h"

DEFINE_bool(ignore_distance, false,
            "If true, ignore distance when shooting and obay joystick_reader");

namespace y2023 {
namespace control_loops {
namespace superstructure {

using ::aos::monotonic_clock;

using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::RelativeEncoderProfiledJointStatus;

Superstructure::Superstructure(
    ::aos::EventLoop *event_loop,
    std::shared_ptr<const constants::Values> values,
    const aos::FlatbufferVector<ArmTrajectories> &arm_trajectories,
    const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      values_(values),
      constants_fetcher_(event_loop),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      arm_(values_, arm_trajectories.message()),
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

  end_effector_.RunIteration(
      timestamp,
      unsafe_goal != nullptr ? unsafe_goal->roller_goal() : RollerGoal::IDLE,
      position->has_roller_falcon()
          ? position->roller_falcon()->torque_current()
          : 0.0,
      position->cone_position(), position->end_effector_cube_beam_break(),
      &output_struct.roller_voltage,
      unsafe_goal != nullptr ? unsafe_goal->preloaded_with_cone() : false);

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();
  status_builder.add_zeroed(wrist_.zeroed() && arm_.zeroed());
  status_builder.add_estopped(wrist_.estopped() || arm_.estopped());
  status_builder.add_arm(arm_status_offset);
  status_builder.add_wrist(wrist_offset);
  status_builder.add_end_effector_state(end_effector_.state());
  // TODO(milind): integrate this with ML game piece detection somehow
  status_builder.add_game_piece(end_effector_.game_piece());
  const std::optional<double> game_piece_position =
      LateralOffsetForTimeOfFlight(position->cone_position());
  if (game_piece_position.has_value()) {
    status_builder.add_game_piece_position(game_piece_position.value());
  }

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

std::optional<double> Superstructure::LateralOffsetForTimeOfFlight(
    double reading) {
  switch (end_effector_.game_piece()) {
    case vision::Class::NONE:
      return std::nullopt;
    case vision::Class::CUBE:
      // Cubes are definitionally centered.
      return 0.0;
    case vision::Class::CONE_UP:
    case vision::Class::CONE_DOWN:
      // execute logic below.
      break;
  }
  constexpr double kInvalidReading = 0.93;
  if (reading > kInvalidReading) {
    return std::nullopt;
  }
  const TimeOfFlight *calibration = CHECK_NOTNULL(
      CHECK_NOTNULL(constants_fetcher_.constants().robot())->tof());
  // TODO(james): Use a generic interpolation table class.
  auto table = CHECK_NOTNULL(calibration->interpolation_table());
  CHECK_EQ(2u, table->size());
  double x1 = table->Get(0)->tof_reading();
  double x2 = table->Get(1)->tof_reading();
  double y1 = table->Get(0)->lateral_position();
  double y2 = table->Get(1)->lateral_position();
  return frc971::shooter_interpolation::Blend((reading - x1) / (x2 - x1), y1,
                                              y2);
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
