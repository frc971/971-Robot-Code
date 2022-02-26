#include "y2022/control_loops/superstructure/superstructure.h"

#include "aos/events/event_loop.h"
#include "y2022/control_loops/superstructure/collision_avoidance.h"

namespace y2022 {
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
      climber_(values_->climber.subsystem_params),
      intake_front_(values_->intake_front.subsystem_params),
      intake_back_(values_->intake_back.subsystem_params),
      turret_(values_->turret.subsystem_params),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      catapult_(*values_) {
  event_loop->SetRuntimeRealtimePriority(30);
}

void Superstructure::RunIteration(const Goal *unsafe_goal,
                                  const Position *position,
                                  aos::Sender<Output>::Builder *output,
                                  aos::Sender<Status>::Builder *status) {
  OutputT output_struct;

  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    intake_front_.Reset();
    intake_back_.Reset();
    turret_.Reset();
    climber_.Reset();
    catapult_.Reset();
  }

  collision_avoidance_.UpdateGoal(
      {.intake_front_position = intake_front_.estimated_position(),
       .intake_back_position = intake_back_.estimated_position(),
       .turret_position = turret_.estimated_position()},
      unsafe_goal);

  turret_.set_min_position(collision_avoidance_.min_turret_goal());
  turret_.set_max_position(collision_avoidance_.max_turret_goal());
  intake_front_.set_min_position(collision_avoidance_.min_intake_front_goal());
  intake_front_.set_max_position(collision_avoidance_.max_intake_front_goal());
  intake_back_.set_min_position(collision_avoidance_.min_intake_back_goal());
  intake_back_.set_max_position(collision_avoidance_.max_intake_back_goal());

  drivetrain_status_fetcher_.Fetch();
  const float velocity = robot_velocity();

  double roller_speed_compensated_front = 0;
  double roller_speed_compensated_back = 0;
  double transfer_roller_speed = 0;

  if (unsafe_goal != nullptr) {
    roller_speed_compensated_front =
        unsafe_goal->roller_speed_front() +
        std::max(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    roller_speed_compensated_back =
        unsafe_goal->roller_speed_back() -
        std::min(velocity * unsafe_goal->roller_speed_compensation(), 0.0);

    transfer_roller_speed = unsafe_goal->transfer_roller_speed();
  }


  const flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      catapult_status_offset = catapult_.Iterate(
          unsafe_goal, position,
          output != nullptr ? &(output_struct.catapult_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<RelativeEncoderProfiledJointStatus>
      climber_status_offset = climber_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->climber() : nullptr,
          position->climber(),
          output != nullptr ? &(output_struct.climber_voltage) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      intake_status_offset_front = intake_front_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake_front() : nullptr,
          position->intake_front(),
          output != nullptr ? &(output_struct.intake_voltage_front) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      intake_status_offset_back = intake_back_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake_back() : nullptr,
          position->intake_back(),
          output != nullptr ? &(output_struct.intake_voltage_back) : nullptr,
          status->fbb());

  flatbuffers::Offset<PotAndAbsoluteEncoderProfiledJointStatus>
      turret_status_offset = turret_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->turret() : nullptr,
          position->turret(),
          output != nullptr ? &(output_struct.turret_voltage) : nullptr,
          status->fbb());

  if (output != nullptr) {
    output_struct.roller_voltage_front = roller_speed_compensated_front;
    output_struct.roller_voltage_back = roller_speed_compensated_back;
    output_struct.transfer_roller_voltage = transfer_roller_speed;

    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
  }

  Status::Builder status_builder = status->MakeBuilder<Status>();

  const bool zeroed = intake_front_.zeroed() && intake_back_.zeroed() &&
                      turret_.zeroed() && climber_.zeroed() && catapult_.zeroed();
  const bool estopped = intake_front_.estopped() || intake_back_.estopped() ||
                        turret_.estopped() || climber_.estopped() || catapult_.estopped();

  status_builder.add_zeroed(zeroed);
  status_builder.add_estopped(estopped);

  status_builder.add_intake_front(intake_status_offset_front);
  status_builder.add_intake_back(intake_status_offset_back);
  status_builder.add_turret(turret_status_offset);
  status_builder.add_climber(climber_status_offset);
  status_builder.add_catapult(catapult_status_offset);
  status_builder.add_solve_time(catapult_.solve_time());
  status_builder.add_mpc_active(catapult_.mpc_active());
  status_builder.add_shot_count(catapult_.shot_count());

  (void)status->Send(status_builder.Finish());
}

double Superstructure::robot_velocity() const {
  return (drivetrain_status_fetcher_.get() != nullptr
              ? drivetrain_status_fetcher_->robot_speed()
              : 0.0);
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
