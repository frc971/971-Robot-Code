#include "y2019/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2019/status_light_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

using frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus;
using frc971::control_loops::AbsoluteEncoderProfiledJointStatus;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      status_light_sender_(
          event_loop->MakeSender<::y2019::StatusLight>("/superstructure")),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      elevator_(constants::GetValues().elevator.subsystem_params),
      wrist_(constants::GetValues().wrist.subsystem_params),
      intake_(constants::GetValues().intake),
      stilts_(constants::GetValues().stilts.subsystem_params) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    elevator_.Reset();
    wrist_.Reset();
    intake_.Reset();
    stilts_.Reset();
  }

  OutputT output_struct;

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      elevator_status_offset = elevator_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->elevator() : nullptr,
          position->elevator(),
          output != nullptr ? &(output_struct.elevator_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      wrist_status_offset = wrist_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->wrist() : nullptr,
          position->wrist(),
          output != nullptr ? &(output_struct.wrist_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<AbsoluteEncoderProfiledJointStatus> intake_status_offset =
      intake_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake() : nullptr,
          position->intake_joint(),
          output != nullptr ? &(output_struct.intake_joint_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      stilts_status_offset = stilts_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->stilts() : nullptr,
          position->stilts(),
          output != nullptr ? &(output_struct.stilts_voltage) : nullptr,
          status->fbb());

  bool has_piece;
  vacuum_.Iterate(unsafe_goal != nullptr ? unsafe_goal->suction() : nullptr,
                  position->suction_pressure(), &output_struct, &has_piece,
                  event_loop());

  bool zeroed;
  bool estopped;
  {
    PotAndAbsoluteEncoderProfiledJointStatus *elevator_status =
        GetMutableTemporaryPointer(*status->fbb(), elevator_status_offset);
    PotAndAbsoluteEncoderProfiledJointStatus *wrist_status =
        GetMutableTemporaryPointer(*status->fbb(), wrist_status_offset);
    AbsoluteEncoderProfiledJointStatus *intake_status =
        GetMutableTemporaryPointer(*status->fbb(), intake_status_offset);
    PotAndAbsoluteEncoderProfiledJointStatus *stilts_status =
        GetMutableTemporaryPointer(*status->fbb(), stilts_status_offset);

    zeroed = elevator_status->zeroed() && wrist_status->zeroed() &&
             intake_status->zeroed() && stilts_status->zeroed();

    estopped = elevator_status->estopped() || wrist_status->estopped() ||
               intake_status->estopped() || stilts_status->estopped();
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);
  status_builder.add_has_piece(has_piece);

  status_builder.add_elevator(elevator_status_offset);
  status_builder.add_wrist(wrist_status_offset);
  status_builder.add_intake(intake_status_offset);
  status_builder.add_stilts(stilts_status_offset);

  flatbuffers::Offset<Status> status_offset = status_builder.Finish();

  Status *status_flatbuffer =
      GetMutableTemporaryPointer(*status->fbb(), status_offset);

  if (output) {
    if (unsafe_goal &&
        status_flatbuffer->intake()->position() > kMinIntakeAngleForRollers) {
      output_struct.intake_roller_voltage = unsafe_goal->roller_voltage();
    } else {
      output_struct.intake_roller_voltage = 0.0;
    }

    output->Send(Output::Pack(*output->fbb(), &output_struct));
  }

  if (unsafe_goal) {
    if (!unsafe_goal->has_suction() || !unsafe_goal->suction()->grab_piece()) {
      wrist_.set_controller_index(0);
      elevator_.set_controller_index(0);
    } else if (unsafe_goal->suction()->gamepiece_mode() == 0) {
      wrist_.set_controller_index(1);
      elevator_.set_controller_index(1);
    } else {
      wrist_.set_controller_index(2);
      elevator_.set_controller_index(2);
    }
  }

  // TODO(theo) move these up when Iterate() is split
  // update the goals
  collision_avoidance_.UpdateGoal(status_flatbuffer, unsafe_goal);

  elevator_.set_min_position(collision_avoidance_.min_elevator_goal());
  wrist_.set_min_position(collision_avoidance_.min_wrist_goal());
  wrist_.set_max_position(collision_avoidance_.max_wrist_goal());
  intake_.set_min_position(collision_avoidance_.min_intake_goal());
  intake_.set_max_position(collision_avoidance_.max_intake_goal());

  drivetrain_status_fetcher_.Fetch();

  if (status && unsafe_goal) {
    // Light Logic
    if (status_flatbuffer->estopped()) {
      // Estop is red
      SendColors(1.0, 0.0, 0.0);
    } else if (drivetrain_status_fetcher_.get() &&
               drivetrain_status_fetcher_->line_follow_logging()->frozen()) {
      // Vision align is flashing white for button pressed, purple for target
      // acquired.
      ++line_blink_count_;
      if (line_blink_count_ < 20) {
        if (drivetrain_status_fetcher_->line_follow_logging()->have_target()) {
          SendColors(1.0, 0.0, 1.0);
        } else {
          SendColors(1.0, 1.0, 1.0);
        }
      } else {
        // And then flash with green if we have a game piece.
        if (status_flatbuffer->has_piece()) {
          SendColors(0.0, 1.0, 0.0);
        } else {
          SendColors(0.0, 0.0, 0.0);
        }
      }

      if (line_blink_count_ > 40) {
        line_blink_count_ = 0;
      }
    } else {
      line_blink_count_ = 0;
      if (status_flatbuffer->has_piece()) {
        // Green if we have a game piece.
        SendColors(0.0, 1.0, 0.0);
      } else if ((!unsafe_goal->has_suction() ||
                  unsafe_goal->suction()->gamepiece_mode() == 0) &&
                 !status_flatbuffer->has_piece()) {
        // Ball mode is orange
        SendColors(1.0, 0.1, 0.0);
      } else if (unsafe_goal->has_suction() &&
                 unsafe_goal->suction()->gamepiece_mode() == 1 &&
                 !status_flatbuffer->has_piece()) {
        // Disk mode is deep blue
        SendColors(0.05, 0.1, 0.5);
      } else {
        SendColors(0.0, 0.0, 0.0);
      }
    }
  }

  status->Send(status_offset);
}

void Superstructure::SendColors(float red, float green, float blue) {
  auto builder = status_light_sender_.MakeBuilder();

  StatusLight::Builder status_light_builder =
      builder.MakeBuilder<StatusLight>();
  status_light_builder.add_red(red);
  status_light_builder.add_green(green);
  status_light_builder.add_blue(blue);

  if (!builder.Send(status_light_builder.Finish())) {
    AOS_LOG(ERROR, "Failed to send lights.\n");
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019
