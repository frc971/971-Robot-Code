#include "y2020/control_loops/superstructure/shooter/shooter.h"

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "y2020/control_loops/superstructure/accelerator/accelerator_plant.h"
#include "y2020/control_loops/superstructure/finisher/finisher_plant.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

Shooter::Shooter()
    : finisher_(
          finisher::MakeIntegralFinisherLoop(), finisher::kBemf,
          // There are 2 motors.  So the current limit per motor is going to be
          // using resistance * 2 to un-parallel the motor resistances.
          finisher::kResistance * 2.0),
      accelerator_left_(accelerator::MakeIntegralAcceleratorLoop(),
                        accelerator::kBemf, accelerator::kResistance),
      accelerator_right_(accelerator::MakeIntegralAcceleratorLoop(),
                         accelerator::kBemf, accelerator::kResistance) {}

void Shooter::UpToSpeed(const ShooterGoal *goal) {
  finisher_ready_ =
      (std::abs(goal->velocity_finisher() - finisher_.avg_angular_velocity()) <
           kVelocityToleranceFinisher &&
       std::abs(goal->velocity_finisher() - finisher_.velocity()) <
           kVelocityToleranceFinisher &&
       goal->velocity_finisher() > kVelocityToleranceFinisher);
  accelerator_ready_ =
      (std::abs(goal->velocity_accelerator() -
                accelerator_left_.avg_angular_velocity()) <
           kVelocityToleranceAccelerator &&
       std::abs(goal->velocity_accelerator() -
                accelerator_right_.avg_angular_velocity()) <
           kVelocityToleranceAccelerator &&
       std::abs(goal->velocity_accelerator() - accelerator_left_.velocity()) <
           kVelocityToleranceAccelerator &&
       std::abs(goal->velocity_accelerator() - accelerator_right_.velocity()) <
           kVelocityToleranceAccelerator &&
       goal->velocity_accelerator() > kVelocityToleranceAccelerator);
}

flatbuffers::Offset<ShooterStatus> Shooter::RunIteration(
    const ShooterGoal *goal, const ShooterPosition *position,
    flatbuffers::FlatBufferBuilder *fbb, OutputT *output,
    const aos::monotonic_clock::time_point position_timestamp) {
  const double last_finisher_velocity = finisher_.velocity();

  // Update position, output, and status for our two shooter sides.
  finisher_.set_position(position->theta_finisher(), position_timestamp);
  accelerator_left_.set_position(position->theta_accelerator_left(),
                                 position_timestamp);
  accelerator_right_.set_position(position->theta_accelerator_right(),
                                  position_timestamp);

  // Update goal.
  if (goal) {
    if (std::abs(goal->velocity_finisher() - finisher_goal()) >=
        kVelocityToleranceFinisher) {
      finisher_goal_changed_ = true;
      last_finisher_velocity_max_ = 0.0;
    }

    finisher_.set_goal(goal->velocity_finisher());
    accelerator_left_.set_goal(goal->velocity_accelerator());
    accelerator_right_.set_goal(goal->velocity_accelerator());
  }

  finisher_.Update(output == nullptr);
  accelerator_left_.Update(output == nullptr);
  accelerator_right_.Update(output == nullptr);

  if (goal) {
    UpToSpeed(goal);
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
  status_builder.add_ready(ready());

  if (finisher_goal_changed_) {
    // If we have caught up to the new goal, we can start detecting if a ball
    // was shot.
    finisher_goal_changed_ = (std::abs(finisher_.velocity() - finisher_goal()) >
                              kVelocityToleranceFinisher);
  }

  if (!finisher_goal_changed_) {
    const bool finisher_was_accelerating = finisher_accelerating_;
    finisher_accelerating_ = (finisher_.velocity() > last_finisher_velocity);
    if (finisher_was_accelerating && !finisher_accelerating_) {
      last_finisher_velocity_max_ = std::min(
          last_finisher_velocity, static_cast<double>(finisher_goal()));
    }

    const double finisher_velocity_dip =
        last_finisher_velocity_max_ - finisher_.velocity();

    if (finisher_velocity_dip < kVelocityToleranceFinisher &&
        ball_in_finisher_) {
      // If we detected a ball in the flywheel and now the angular velocity has
      // come back up close to the last local maximum or is greater than it, the
      // ball has been shot.
      balls_shot_++;
      VLOG(1) << "Shot ball at " << position_timestamp;
      ball_in_finisher_ = false;
    } else if (!ball_in_finisher_ &&
               (finisher_goal() > kVelocityToleranceFinisher)) {
      // There is probably a ball in the flywheel if the angular
      // velocity is atleast kMinVelocityErrorWithBall less than the last local
      // maximum.
      ball_in_finisher_ =
          (finisher_velocity_dip >= kMinFinisherVelocityDipWithBall);
    }
  }

  status_builder.add_balls_shot(balls_shot_);

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
