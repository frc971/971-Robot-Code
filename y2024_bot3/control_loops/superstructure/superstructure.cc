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
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      arm_(robot_constants_->common()->arm(),
           robot_constants_->robot()->arm_constants()->zeroing_constants()) {
  event_loop->SetRuntimeRealtimePriority(30);
}

bool PositionNear(double position, double goal, double threshold) {
  return std::abs(position - goal) < threshold;
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

  aos::FlatbufferFixedAllocatorArray<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal, 512>
      arm_goal_buffer;

  ArmStatus arm_status = ArmStatus::IDLE;
  IntakeRollerStatus intake_roller_status = IntakeRollerStatus::NONE;
  double arm_position =
      robot_constants_->robot()->arm_constants()->arm_positions()->idle();
  double roller_voltage = 0.0;
  if (unsafe_goal != nullptr) {
    switch (unsafe_goal->roller_goal()) {
      case IntakeGoal::INTAKE:
        roller_voltage = position->intake_beambreak()
                             ? 0.0
                             : robot_constants_->common()
                                   ->intake_voltages()
                                   ->operating_voltage();
        intake_roller_status = position->intake_beambreak()
                                   ? IntakeRollerStatus::NONE
                                   : IntakeRollerStatus::INTAKE;
        break;
      case IntakeGoal::SCORE:
        intake_roller_status = IntakeRollerStatus::SCORE;
        roller_voltage =
            robot_constants_->common()->intake_voltages()->operating_voltage();
        break;
      case IntakeGoal::SPIT:
        intake_roller_status = IntakeRollerStatus::SPIT;
        roller_voltage =
            robot_constants_->common()->intake_voltages()->spitting_voltage();
        break;
      case IntakeGoal::NONE:
        roller_voltage = 0.0;
        break;
    }

    switch (unsafe_goal->arm_position()) {
      case PivotGoal::INTAKE:
        arm_status = ArmStatus::INTAKE;
        arm_position = robot_constants_->robot()
                           ->arm_constants()
                           ->arm_positions()
                           ->intake();
        break;
      case PivotGoal::AMP:
        arm_status = ArmStatus::AMP;
        arm_position =
            robot_constants_->robot()->arm_constants()->arm_positions()->amp();
        break;
      default:
        arm_position =
            robot_constants_->robot()->arm_constants()->arm_positions()->idle();
    }
  }
  output_struct.roller_voltage = roller_voltage;

  arm_goal_buffer.Finish(
      frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *arm_goal_buffer.fbb(), arm_position));

  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *arm_goal = &arm_goal_buffer.message();

  // static_zeroing_single_dof_profiled_subsystem.h
  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      arm_offset =
          arm_.Iterate(arm_goal, position->arm(),
                       output != nullptr ? &output_struct.arm_voltage : nullptr,
                       status->fbb());

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = arm_.zeroed();
  const bool estopped = arm_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_arm(arm_offset);
  status_builder.add_arm_status(arm_status);
  status_builder.add_intake_roller_status(intake_roller_status);

  (void)status->Send(status_builder.Finish());
}
}  // namespace y2024_bot3::control_loops::superstructure
