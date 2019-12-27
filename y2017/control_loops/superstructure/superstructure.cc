#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/column/column.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/intake/intake.h"
#include "y2017/control_loops/superstructure/shooter/shooter.h"
#include "y2017/vision/vision_generated.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr double kMaxIntakeRollerVoltage = 12.0;
constexpr double kMaxIndexerRollerVoltage = 12.0;
}  // namespace

typedef ::y2017::constants::Values::ShotParams ShotParams;

Superstructure::Superstructure(::aos::EventLoop *event_loop,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2017::vision::VisionStatus>("/vision")),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      column_(event_loop) {
  shot_interpolation_table_ =
      ::frc971::shooter_interpolation::InterpolationTable<ShotParams>({
          // { distance_to_target, { shot_angle, shot_power, indexer_velocity
          // }},
          {1.21, {0.29, 301.0, -1.0 * M_PI}},   // table entry
          {1.55, {0.305, 316.0, -1.1 * M_PI}},  // table entry
          {1.82, {0.33, 325.0, -1.3 * M_PI}},   // table entry
          {2.00, {0.34, 328.0, -1.4 * M_PI}},   // table entry
          {2.28, {0.36, 338.0, -1.5 * M_PI}},   // table entry
          {2.55, {0.395, 342.0, -1.8 * M_PI}},  // table entry
          {2.81, {0.41, 354.0, -1.90 * M_PI}},  // table entry
          // The following entry is wrong, but will make it so we keep shooting
          // in auto.
          {3.20, {0.41, 354.0, -1.90 * M_PI}},  // table entry
      });
}

void Superstructure::RunIteration(
    const Goal *unsafe_goal,
    const Position *position,
    aos::Sender<Output>::Builder *output,
    aos::Sender<Status>::Builder *status) {
  OutputT output_struct;
  const ::aos::monotonic_clock::time_point monotonic_now =
      event_loop()->monotonic_now();
  if (WasReset()) {
    AOS_LOG(ERROR, "WPILib reset, restarting\n");
    hood_.Reset();
    intake_.Reset();
    shooter_.Reset();
    column_.Reset();
  }

  const vision::VisionStatus *vision_status = nullptr;
  if (vision_status_fetcher_.Fetch()) {
    vision_status = vision_status_fetcher_.get();
  }

  // Create a copy of the goals so that we can modify them.
  double hood_goal_angle = 0.0;
  ShooterGoalT shooter_goal;
  IndexerGoalT indexer_goal;
  bool in_range = true;
  double vision_distance = 0.0;
  if (unsafe_goal != nullptr) {
    hood_goal_angle = unsafe_goal->hood()->angle();
    shooter_goal.angular_velocity = unsafe_goal->shooter()->angular_velocity();
    indexer_goal.angular_velocity = unsafe_goal->indexer()->angular_velocity();
    indexer_goal.voltage_rollers = unsafe_goal->indexer()->voltage_rollers();

    if (!unsafe_goal->use_vision_for_shots()) {
      distance_average_.Reset();
    }

    distance_average_.Tick(monotonic_now, vision_status);
    vision_distance = distance_average_.Get();

    // If we are moving too fast, disable shooting and clear the accumulator.
    double robot_velocity = 0.0;
    drivetrain_status_fetcher_.Fetch();
    if (drivetrain_status_fetcher_.get()) {
      robot_velocity = drivetrain_status_fetcher_->robot_speed();
    }

    if (::std::abs(robot_velocity) > 0.2) {
      if (unsafe_goal->use_vision_for_shots()) {
        AOS_LOG(INFO, "Moving too fast, resetting\n");
      }
      distance_average_.Reset();
    }
    if (distance_average_.Valid()) {
      if (unsafe_goal->use_vision_for_shots()) {
        ShotParams shot_params;
        if (shot_interpolation_table_.GetInRange(
                distance_average_.Get(), &shot_params)) {
          hood_goal_angle = shot_params.angle;
          shooter_goal.angular_velocity = shot_params.power;
          if (indexer_goal.angular_velocity != 0.0) {
            indexer_goal.angular_velocity = shot_params.indexer_velocity;
          }
        } else {
          in_range = false;
        }
      }
      AOS_LOG(
          DEBUG, "VisionDistance %f, hood %f shooter %f, indexer %f * M_PI\n",
          vision_distance, hood_goal_angle,
          shooter_goal.angular_velocity, indexer_goal.angular_velocity / M_PI);
    } else {
      AOS_LOG(DEBUG, "VisionNotValid %f\n", vision_distance);
      if (unsafe_goal->use_vision_for_shots()) {
        in_range = false;
        indexer_goal.angular_velocity = 0.0;
      }
    }
  }

  flatbuffers::Offset<frc971::control_loops::IndexProfiledJointStatus>
      hood_offset = hood_.Iterate(
          monotonic_now, unsafe_goal != nullptr ? &hood_goal_angle : nullptr,
          unsafe_goal != nullptr ? unsafe_goal->hood()->profile_params()
                                 : nullptr,
          position->hood(),
          output != nullptr ? &(output_struct.voltage_hood) : nullptr,
          status->fbb());
  flatbuffers::Offset<ShooterStatus> shooter_offset = shooter_.Iterate(
      unsafe_goal != nullptr ? &shooter_goal : nullptr,
      position->theta_shooter(), position_context().monotonic_event_time,
      output != nullptr ? &(output_struct.voltage_shooter) : nullptr,
      status->fbb());

  // Implement collision avoidance by passing down a freeze or range restricting
  // signal to the column and intake objects.

  // Wait until the column is ready before doing collision avoidance.
  if (unsafe_goal && column_.state() == column::Column::State::RUNNING) {
    if (!ignore_collisions_) {
      // The turret is in a position (or wants to be in a position) where we
      // need the intake out.  Push it out.
      const bool column_goal_not_safe =
          unsafe_goal->turret()->angle() > column::Column::kTurretMax ||
          unsafe_goal->turret()->angle() < column::Column::kTurretMin;
      const bool column_position_not_safe =
          column_.turret_position() > column::Column::kTurretMax ||
          column_.turret_position() < column::Column::kTurretMin;

      if (column_goal_not_safe || column_position_not_safe) {
        intake_.set_min_position(column::Column::kIntakeZeroingMinDistance);
      } else {
        intake_.clear_min_position();
      }
      // The intake is in a position where it could hit.  Don't move the turret.
      if (intake_.position() < column::Column::kIntakeZeroingMinDistance -
                                   column::Column::kIntakeTolerance &&
          column_position_not_safe) {
        column_.set_freeze(true);
      } else {
        column_.set_freeze(false);
      }
    } else {
      // If we are ignoring collisions, unfreeze and un-limit the min.
      column_.set_freeze(false);
      intake_.clear_min_position();
    }
  } else {
    column_.set_freeze(false);
  }

  // Make some noise if someone left this set...
  if (ignore_collisions_) {
    AOS_LOG(ERROR, "Collisions ignored\n");
  }

  flatbuffers::Offset<
      ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
      intake_offset = intake_.Iterate(
          unsafe_goal != nullptr ? unsafe_goal->intake() : nullptr,
          position->intake(),
          output != nullptr ? &(output_struct.voltage_intake) : nullptr,
          status->fbb());

  std::pair<flatbuffers::Offset<IndexerStatus>,
            flatbuffers::Offset<TurretProfiledSubsystemStatus>>
      indexer_and_turret_offsets = column_.Iterate(
          monotonic_now, unsafe_goal != nullptr ? &indexer_goal : nullptr,
          unsafe_goal != nullptr ? unsafe_goal->turret() : nullptr,
          position->column(), vision_status,
          output != nullptr ? &(output_struct.voltage_indexer) : nullptr,
          output != nullptr ? &(output_struct.voltage_turret) : nullptr,
          status->fbb(), &intake_);

  const frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus
      *temp_intake_status = GetTemporaryPointer(*status->fbb(), intake_offset);
  const frc971::control_loops::IndexProfiledJointStatus *temp_hood_status =
      GetTemporaryPointer(*status->fbb(), hood_offset);
  const TurretProfiledSubsystemStatus *temp_turret_status =
      GetTemporaryPointer(*status->fbb(), indexer_and_turret_offsets.second);

  const bool estopped = temp_intake_status->estopped() ||
                        temp_hood_status->estopped() ||
                        temp_turret_status->estopped();
  const bool zeroed = temp_intake_status->zeroed() &&
                      temp_hood_status->zeroed() &&
                      temp_turret_status->zeroed();
  const bool turret_vision_tracking = temp_turret_status->vision_tracking();

  Status::Builder status_builder = status->MakeBuilder<Status>();
  status_builder.add_intake(intake_offset);
  status_builder.add_hood(hood_offset);
  status_builder.add_shooter(shooter_offset);
  status_builder.add_turret(indexer_and_turret_offsets.second);
  status_builder.add_indexer(indexer_and_turret_offsets.first);

  status_builder.add_estopped(estopped);
  status_builder.add_zeroed(zeroed);

  status_builder.add_vision_distance(vision_distance);

  if (output && unsafe_goal) {
    output_struct.gear_servo =
        ::std::min(1.0, ::std::max(0.0, unsafe_goal->intake()->gear_servo()));

    output_struct.voltage_intake_rollers =
        ::std::max(-kMaxIntakeRollerVoltage,
                   ::std::min(unsafe_goal->intake()->voltage_rollers(),
                              kMaxIntakeRollerVoltage));
    output_struct.voltage_indexer_rollers =
        ::std::max(-kMaxIndexerRollerVoltage,
                   ::std::min(unsafe_goal->indexer()->voltage_rollers(),
                              kMaxIndexerRollerVoltage));

    // Set the lights on or off
    output_struct.lights_on = unsafe_goal->lights_on();

    if (estopped) {
      output_struct.red_light_on = true;
      output_struct.green_light_on = false;
      output_struct.blue_light_on = false;
    } else if (!zeroed) {
      output_struct.red_light_on = false;
      output_struct.green_light_on = false;
      output_struct.blue_light_on = true;
    } else if (turret_vision_tracking && in_range) {
      output_struct.red_light_on = false;
      output_struct.green_light_on = true;
      output_struct.blue_light_on = false;
    }
  }

  if (output) {
    output->Send(Output::Pack(*output->fbb(), &output_struct));
  }

  status->Send(status_builder.Finish());
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
