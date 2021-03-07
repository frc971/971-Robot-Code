#include "y2020/control_loops/superstructure/shooter/shooter.h"

#include <chrono>

#include "aos/logging/logging.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

namespace {
const double kVelocityTolerance = 20.0;
}  // namespace

Shooter::Shooter()
    : finisher_(finisher::MakeIntegralFinisherLoop(), finisher::kBemf,
                finisher::kResistance),
      accelerator_left_(accelerator::MakeIntegralAcceleratorLoop(),
                        accelerator::kBemf, accelerator::kResistance),
      accelerator_right_(accelerator::MakeIntegralAcceleratorLoop(),
                         accelerator::kBemf, accelerator::kResistance) {}

bool Shooter::UpToSpeed(const ShooterGoal *goal) {
  return (
      std::abs(goal->velocity_finisher() - finisher_.avg_angular_velocity()) <
          kVelocityTolerance &&
      std::abs(goal->velocity_accelerator() -
               accelerator_left_.avg_angular_velocity()) < kVelocityTolerance &&
      std::abs(goal->velocity_accelerator() -
               accelerator_right_.avg_angular_velocity()) <
          kVelocityTolerance &&
      std::abs(goal->velocity_finisher() - finisher_.velocity()) <
          kVelocityTolerance &&
      std::abs(goal->velocity_accelerator() - accelerator_left_.velocity()) <
          kVelocityTolerance &&
      std::abs(goal->velocity_accelerator() - accelerator_right_.velocity()) <
          kVelocityTolerance);
}

flatbuffers::Offset<ShooterStatus> Shooter::RunIteration(
    const ShooterGoal *goal, const ShooterPosition *position,
    flatbuffers::FlatBufferBuilder *fbb, OutputT *output,
    const aos::monotonic_clock::time_point position_timestamp) {
  // Update position, output, and status for our two shooter sides.
  finisher_.set_position(position->theta_finisher(), position_timestamp);
  accelerator_left_.set_position(position->theta_accelerator_left(),
                                 position_timestamp);
  accelerator_right_.set_position(position->theta_accelerator_right(),
                                  position_timestamp);

  // Update goal.
  if (goal) {
    finisher_.set_goal(goal->velocity_finisher());
    accelerator_left_.set_goal(goal->velocity_accelerator());
    accelerator_right_.set_goal(goal->velocity_accelerator());
  }

  finisher_.Update(output == nullptr);
  accelerator_left_.Update(output == nullptr);
  accelerator_right_.Update(output == nullptr);

  if (goal) {
    if (UpToSpeed(goal) && goal->velocity_finisher() > kVelocityTolerance &&
        goal->velocity_accelerator() > kVelocityTolerance) {
      ready_ = true;
    } else {
      ready_ = false;
    }
  }

  flatbuffers::Offset<FlywheelControllerStatus> finisher_status_offset =
      finisher_.SetStatus(fbb);
  flatbuffers::Offset<FlywheelControllerStatus> accelerator_left_status_offset =
      accelerator_left_.SetStatus(fbb);
  flatbuffers::Offset<FlywheelControllerStatus>
      accelerator_right_status_offset = accelerator_right_.SetStatus(fbb);

  ShooterStatusBuilder status_builder(*fbb);

  status_builder.add_finisher(finisher_status_offset);
  status_builder.add_accelerator_left(accelerator_left_status_offset);
  status_builder.add_accelerator_right(accelerator_right_status_offset);

  if (output) {
    output->finisher_voltage = finisher_.voltage();
    output->accelerator_left_voltage = accelerator_left_.voltage();
    output->accelerator_right_voltage = accelerator_right_.voltage();
  }

  return status_builder.Finish();
}

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
