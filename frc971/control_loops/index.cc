#include "frc971/control_loops/index.h"

#include <stdio.h>

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/index_motor_plant.h"

using ::aos::time::Time;

namespace frc971 {
namespace control_loops {

IndexMotor::IndexMotor(control_loops::IndexLoop *my_index)
    : aos::control_loops::ControlLoop<control_loops::IndexLoop>(my_index),
      wrist_loop_(new StateFeedbackLoop<2, 1, 1>(MakeIndexLoop())),
      hopper_disc_count_(0),
      total_disc_count_(0),
      loader_up_(false),
      disc_clamped_(false),
      disc_ejected_(false),
      last_bottom_disc_detect_(false) {
}

const /*static*/ double IndexMotor::kDiscRadius = 10.875 * 0.0254 / 2;
const /*static*/ double IndexMotor::kRollerRadius = 2.0 * 0.0254 / 2;

bool IndexMotor::FetchConstants() {
  if (!constants::horizontal_lower_limit(&horizontal_lower_limit_)) {
    LOG(ERROR, "Failed to fetch the horizontal lower limit constant.\n");
    return false;
  }
  if (!constants::horizontal_upper_limit(&horizontal_upper_limit_)) {
    LOG(ERROR, "Failed to fetch the horizontal upper limit constant.\n");
    return false;
  }
  if (!constants::horizontal_hall_effect_start_angle(
          &horizontal_hall_effect_start_angle_)) {
    LOG(ERROR, "Failed to fetch the horizontal start angle constant.\n");
    return false;
  }
  if (!constants::horizontal_zeroing_speed(
          &horizontal_zeroing_speed_)) {
    LOG(ERROR, "Failed to fetch the horizontal zeroing speed constant.\n");
    return false;
  }

  return true;
}

// Distance to move the indexer when grabbing a disc.
const double kNextPosition = 10.0;

/*static*/ double IndexMotor::ConvertDiscAngleToIndex(const double angle) {
  return (angle * (1 + (kDiscRadius * 2 + kRollerRadius) / kRollerRadius));
}

/*static*/ double IndexMotor::ConvertDiscAngleToDiscPosition(const double angle) {
  return angle * (kDiscRadius + kRollerRadius);
}

/*static*/ double IndexMotor::ConvertIndexToDiscAngle(const double angle) {
  return (angle / (1 + (kDiscRadius * 2 + kRollerRadius) / kRollerRadius));
}

/*static*/ double IndexMotor::ConvertIndexToDiscPosition(const double angle) {
  return IndexMotor::ConvertDiscAngleToDiscPosition(
      ConvertIndexToDiscAngle(angle));
}

// Positive angle is towards the shooter, and positive power is towards the
// shooter.
void IndexMotor::RunIteration(
    const control_loops::IndexLoop::Goal *goal,
    const control_loops::IndexLoop::Position *position,
    control_loops::IndexLoop::Output *output,
    control_loops::IndexLoop::Status *status) {
  // Make goal easy to work with.
  Goal goal_enum = static_cast<Goal>(goal->goal_state);

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->transfer_voltage = 0.0;
    output->index_voltage = 0.0;
  }

  status->ready_to_intake = false;

  // Cache the constants to avoid error handling down below.
  if (!FetchConstants()) {
    return;
  }

  if (position) {
    wrist_loop_->Y << position->index_position;
  }
  const double index_position = wrist_loop_->X_hat(0, 0);

  bool safe_to_change_state_ = true;
  switch (safe_goal_) {
    case HOLD:
      // The goal should already be good, so sit tight with everything the same
      // as it was.
      printf("HOLD Not implemented\n");
      break;
    case READY_LOWER:
      printf("READY_LOWER Not implemented\n");
      break;
    case INTAKE:
      {
        Time now = Time::Now();
        if (hopper_disc_count_ < 4) {
          output->transfer_voltage = 12.0;
        }
        // Posedge of the disc entering the beam break.
        if (position) {
          if (position->bottom_disc_detect && !last_bottom_disc_detect_) {
            transfer_frisbee_.Reset();
            transfer_frisbee_.bottom_posedge_time_ = now;
            printf("Posedge of bottom disc %f\n",
                   transfer_frisbee_.bottom_posedge_time_.ToSeconds());
            ++hopper_disc_count_;
          }

          // Disc exited the beam break now.
          if (!position->bottom_disc_detect && last_bottom_disc_detect_) {
            transfer_frisbee_.bottom_negedge_time_ = now;
            printf("Negedge of bottom disc %f\n",
                   transfer_frisbee_.bottom_negedge_time_.ToSeconds());
            frisbees_.push_front(transfer_frisbee_);
          }

          if (position->bottom_disc_detect) {
            output->transfer_voltage = 12.0;
            // Must wait until the disc gets out before we can change state.
            safe_to_change_state_ = false;

            // TODO(aschuh): A disc on the way through needs to start moving the
            // indexer if it isn't already moving.  Maybe?

            Time elapsed_posedge_time = now -
                transfer_frisbee_.bottom_posedge_time_;
            if (elapsed_posedge_time >= Time::InSeconds(0.3)) {
              // It has been too long.  The disc must be jammed.
              LOG(ERROR, "Been way too long.  Jammed disc?\n");
              printf("Been way too long.  Jammed disc?\n");
            }
          }

          for (Frisbee &frisbee : frisbees_) {
            if (!frisbee.has_been_indexed_) {
              output->transfer_voltage = 12.0;
              Time elapsed_posedge_time = now -
                  frisbee.bottom_posedge_time_;
              if (elapsed_posedge_time >= Time::InSeconds(0.07)) {
                // Should have just engaged.
                // Save the indexer position, and the time.

                // It has been long enough since the disc entered the indexer.
                // Treat now as the time at which it contacted the indexer.
                LOG(INFO, "Grabbed on the index now at %f\n", index_position);
                printf("Grabbed on the index now at %f\n", index_position);
                frisbee.has_been_indexed_ = true;
                frisbee.index_start_position_ = index_position;
                frisbee.index_start_time_ = now;
              }
            }
            if (!frisbee.has_been_indexed_) {
              // Discs must all be indexed before it is safe to stop indexing.
              safe_to_change_state_ = false;
            }
          }

          double new_index_position = wrist_loop_->R(0, 0);

          // TODO(aschuh): As we loop through, assess the state of the indexer
          // and figure if the bottom disc is in a place such that we can
          // intake without filling the hopper early.
          // status->ready_to_intake = false;

          for (Frisbee &frisbee : frisbees_) {
            if (frisbee.has_been_indexed_) {
              // We want to store it pi from where the disc was grabbed
              // (for now).
              new_index_position = ::std::max(
                  new_index_position,
                  (frisbee.index_start_position_ +
                   ConvertDiscAngleToIndex(M_PI)));
              // TODO(aschuh): We should be able to pick the M_PI knowing if
              // the next disc is coming in hot or not.
            }
          }
          wrist_loop_->R << new_index_position, 0.0;
        }
        printf("INTAKE Not implemented\n");
      }
      break;
    case READY_SHOOTER:
      printf("READY_SHOOTER Not implemented\n");
      break;
    case SHOOT:
      printf("SHOOT Not implemented\n");
      break;
  }

  // Update the observer.
  wrist_loop_->Update(position != NULL, output == NULL);

  if (position) {
    LOG(DEBUG, "pos=%f currently %f\n",
        position->index_position, index_position);
    last_bottom_disc_detect_ = position->bottom_disc_detect;
  }

  status->hopper_disc_count = hopper_disc_count_;
  status->total_disc_count = total_disc_count_;


  if (output) {
    output->index_voltage = wrist_loop_->U(0, 0);
  }

  if (safe_to_change_state_) {
    safe_goal_ = goal_enum;
  }
}

}  // namespace control_loops
}  // namespace frc971
