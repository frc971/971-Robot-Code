#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/column/column.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/intake/intake.h"
#include "y2017/control_loops/superstructure/shooter/shooter.h"
#include "y2017/vision/vision.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

namespace {
// The maximum voltage the intake roller will be allowed to use.
constexpr double kMaxIntakeRollerVoltage = 12.0;
constexpr double kMaxIndexerRollerVoltage = 12.0;
constexpr double kTurretTuckAngle = M_PI / 2.0;
}  // namespace

typedef ::y2017::constants::Values::ShotParams ShotParams;
using ::frc971::control_loops::drivetrain_queue;

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {
  shot_interpolation_table_ =
      ::frc971::shooter_interpolation::InterpolationTable<ShotParams>({
          // { distance_to_target, { shot_angle, shot_power, indexer_velocity }},
          {1.21, {0.29, 301.0, -1.0 * M_PI}},   // table entry
          {1.55, {0.305, 316.0, -1.1 * M_PI}},   // table entry
          {1.82, {0.33, 325.0, -1.3 * M_PI}},   // table entry
          {2.00, {0.34, 328.0, -1.4 * M_PI}},   // table entry
          {2.28, {0.36, 338.0, -1.5 * M_PI}},   // table entry
          {2.55, {0.395, 342.0, -1.8 * M_PI}},  // table entry
          {2.81, {0.41, 351.0, -1.90 * M_PI}},  // table entry
      });
}

void Superstructure::RunIteration(
    const control_loops::SuperstructureQueue::Goal *unsafe_goal,
    const control_loops::SuperstructureQueue::Position *position,
    control_loops::SuperstructureQueue::Output *output,
    control_loops::SuperstructureQueue::Status *status) {
  if (WasReset()) {
    LOG(ERROR, "WPILib reset, restarting\n");
    hood_.Reset();
    intake_.Reset();
    shooter_.Reset();
    column_.Reset();
  }

  const vision::VisionStatus *vision_status = nullptr;
  if (vision::vision_status.FetchLatest()) {
    vision_status = vision::vision_status.get();
  }

  // Create a copy of the goals so that we can modify them.
  HoodGoal hood_goal;
  ShooterGoal shooter_goal;
  IndexerGoal indexer_goal;
  bool in_range = true;
  if (unsafe_goal != nullptr) {
    hood_goal = unsafe_goal->hood;
    shooter_goal = unsafe_goal->shooter;
    indexer_goal = unsafe_goal->indexer;

    if (!unsafe_goal->use_vision_for_shots) {
      distance_average_.Reset();
    }

    distance_average_.Tick(::aos::monotonic_clock::now(), vision_status);
    status->vision_distance = distance_average_.Get();

    // If we are moving too fast, disable shooting and clear the accumulator.
    double robot_velocity = 0.0;
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      robot_velocity = drivetrain_queue.status->robot_speed;
    }

    if (::std::abs(robot_velocity) > 0.2) {
      if (unsafe_goal->use_vision_for_shots) {
        LOG(INFO, "Moving too fast, resetting\n");
      }
      distance_average_.Reset();
    }
    if (distance_average_.Valid()) {
      if (unsafe_goal->use_vision_for_shots) {
        ShotParams shot_params;
        if (shot_interpolation_table_.GetInRange(
                distance_average_.Get(), &shot_params)) {
          hood_goal.angle = shot_params.angle;
          shooter_goal.angular_velocity = shot_params.power;
          if (indexer_goal.angular_velocity != 0.0) {
            indexer_goal.angular_velocity = shot_params.indexer_velocity;
          }
        } else {
          in_range = false;
        }
      }
      LOG(DEBUG, "VisionDistance %f, hood %f shooter %f, indexer %f * M_PI\n",
          status->vision_distance, hood_goal.angle,
          shooter_goal.angular_velocity, indexer_goal.angular_velocity / M_PI);
    } else {
      LOG(DEBUG, "VisionNotValid %f\n", status->vision_distance);
      if (unsafe_goal->use_vision_for_shots) {
        in_range = false;
        indexer_goal.angular_velocity = 0.0;
      }
    }
  }

  hood_.Iterate(
      unsafe_goal != nullptr ? &hood_goal : nullptr, &(position->hood),
      output != nullptr ? &(output->voltage_hood) : nullptr, &(status->hood));
  shooter_.Iterate(unsafe_goal != nullptr ? &shooter_goal : nullptr,
                   &(position->theta_shooter), position->sent_time,
                   output != nullptr ? &(output->voltage_shooter) : nullptr,
                   &(status->shooter));

  // Implement collision avoidance by passing down a freeze or range restricting
  // signal to the column and intake objects.

  // Wait until the column is ready before doing collision avoidance.
  if (unsafe_goal && column_.state() == column::Column::State::RUNNING) {
    if (!ignore_collisions_) {
      // The turret is in a position (or wants to be in a position) where we
      // need the intake out.  Push it out.
      const bool column_goal_not_safe =
          unsafe_goal->turret.angle > column::Column::kTurretMax ||
          unsafe_goal->turret.angle < column::Column::kTurretMin;
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
    LOG(ERROR, "Collisions ignored\n");
  }

  intake_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->intake) : nullptr,
                  &(position->intake),
                  output != nullptr ? &(output->voltage_intake) : nullptr,
                  &(status->intake));

  column_.Iterate(unsafe_goal != nullptr ? &indexer_goal : nullptr,
                  unsafe_goal != nullptr ? &(unsafe_goal->turret) : nullptr,
                  &(position->column), vision_status,
                  output != nullptr ? &(output->voltage_indexer) : nullptr,
                  output != nullptr ? &(output->voltage_turret) : nullptr,
                  &(status->indexer), &(status->turret), &intake_);

  status->estopped =
      status->intake.estopped | status->hood.estopped | status->turret.estopped;

  status->zeroed =
      status->intake.zeroed && status->hood.zeroed && status->turret.zeroed;

  if (output && unsafe_goal) {
    output->gear_servo =
        ::std::min(1.0, ::std::max(0.0, unsafe_goal->intake.gear_servo));

    output->voltage_intake_rollers =
        ::std::max(-kMaxIntakeRollerVoltage,
                   ::std::min(unsafe_goal->intake.voltage_rollers,
                              kMaxIntakeRollerVoltage));
    output->voltage_indexer_rollers =
        ::std::max(-kMaxIndexerRollerVoltage,
                   ::std::min(unsafe_goal->indexer.voltage_rollers,
                              kMaxIndexerRollerVoltage));

    // Set the lights on or off
    output->lights_on = unsafe_goal->lights_on;

    if (status->estopped) {
      output->red_light_on = true;
      output->green_light_on = false;
      output->blue_light_on = false;
    } else if (!status->zeroed) {
      output->red_light_on = false;
      output->green_light_on = false;
      output->blue_light_on = true;
    } else if (status->turret.vision_tracking && in_range) {
      output->red_light_on = false;
      output->green_light_on = true;
      output->blue_light_on = false;
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
