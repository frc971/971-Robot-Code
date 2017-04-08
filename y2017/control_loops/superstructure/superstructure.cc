#include "y2017/control_loops/superstructure/superstructure.h"

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
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
}  // namespace

Superstructure::Superstructure(
    control_loops::SuperstructureQueue *superstructure_queue)
    : aos::controls::ControlLoop<control_loops::SuperstructureQueue>(
          superstructure_queue) {}

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

  hood_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->hood) : nullptr,
                &(position->hood),
                output != nullptr ? &(output->voltage_hood) : nullptr,
                &(status->hood));
  shooter_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->shooter) : nullptr,
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
      if (::std::abs(unsafe_goal->turret.angle) >
              column::Column::kTurretNearZero ||
          ::std::abs(column_.turret_position()) >
              column::Column::kTurretNearZero) {
        intake_.set_min_position(column::Column::kIntakeZeroingMinDistance);
      } else {
        intake_.clear_min_position();
      }
      // The intake is in a position where it could hit.  Don't move the turret.
      if (intake_.position() < column::Column::kIntakeZeroingMinDistance -
                                   column::Column::kIntakeTolerance &&
          ::std::abs(column_.turret_position()) >
              column::Column::kTurretNearZero) {
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

  column_.Iterate(unsafe_goal != nullptr ? &(unsafe_goal->indexer) : nullptr,
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
    } else if (status->turret.vision_tracking) {
      output->red_light_on = false;
      output->green_light_on = true;
      output->blue_light_on = false;
    } else if (!status->zeroed) {
      output->red_light_on = false;
      output->green_light_on = false;
      output->blue_light_on = true;
    }
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017
