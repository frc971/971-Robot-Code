#ifndef FRC971_CONTROL_LOOPS_HALL_EFFECT_INL_H_
#define FRC971_CONTROL_LOOPS_HALL_EFFECT_INL_H_

#include "frc971/control_loops/hall_effect_loop.h"

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/logging/logging.h"

namespace frc971 {
namespace control_loops {

template <int kNumHallEffect>
HallEffectLoop<kNumHallEffect>::HallEffectLoop(
    StateFeedbackLoop<2, 1, 1>* state_feedback_loop,
    bool zero_down, double max_zeroing_voltage)
  : kMaxZeroingVoltage(max_zeroing_voltage),
    zero_down_(zero_down),
    state_(UNINITIALIZED),
    loop_(state_feedback_loop),
    current_position_(0.0),
    last_off_position_(0.0),
    last_calibration_sensor_(-1),
    zeroing_position_(0.0),
    zero_offset_(0.0),
    old_zero_offset_(0.0) {
}

template <int kNumHallEffect>
int HallEffectLoop<kNumHallEffect>::HallEffect() const {
  for (int i = 0; i < kNumHallEffect; ++i) {
    if (hall_effect_[i]) {
      return i;
    }
  }
  return -1;
}

template <int kNumHallEffect>
int HallEffectLoop<kNumHallEffect>::WhichHallEffect() const {
  if (zero_down_) {
    for (int i = 0; i < kNumHallEffect; ++i) {
      if (calibration_[i] + hall_effect_angle_[i] < last_off_position_ &&
          calibration_[i] + hall_effect_angle_[i] >= current_position_) {
        return i;
      }
    }
  } else {
    for (int i = 0; i < kNumHallEffect; ++i) {
      if (calibration_[i] + hall_effect_angle_[i] > last_off_position_ &&
          calibration_[i] + hall_effect_angle_[i] <= current_position_) {
        return i;
      }
    }
  }
  return -1;
}

template <int kNumHallEffect>
void HallEffectLoop<kNumHallEffect>::LimitZeroingGoal() {
  if (loop_->U_uncapped(0, 0) > kMaxZeroingVoltage) {
    double excess = (loop_->U_uncapped(0, 0) - kMaxZeroingVoltage)
                    / loop_->K(0, 0);
    zeroing_position_ -= excess;
  }
  if (loop_->U_uncapped(0, 0) < -kMaxZeroingVoltage) {
    double excess = (loop_->U_uncapped(0, 0) + kMaxZeroingVoltage)
                    / loop_->K(0, 0);
    zeroing_position_ -= excess;
  }
}

template <int kNumHallEffect>
void HallEffectLoop<kNumHallEffect>::UpdateZeros(
    ::std::array<double, kNumHallEffect> hall_effect_angle,
    ::std::array<bool, kNumHallEffect> hall_effect,
    ::std::array<double, kNumHallEffect> calibration,
    double zeroing_speed,
    double position, bool good_position) {
  hall_effect_angle_ = hall_effect_angle;
  hall_effect_ = hall_effect;
  calibration_ = calibration;
  zeroing_speed_ = zeroing_speed;
  current_position_ = position;

  if (!zero_down_) {
    zeroing_speed_ *= -1;
  }

  // Deal with getting all the position variables updated.
  absolute_position_ = current_position_;
  if (good_position) {
    if (state_ == READY) {
      absolute_position_ -= zero_offset_;
    }
    loop_->Y << absolute_position_;
    if (HallEffect() == -1) {
      last_off_position_ = current_position_;
    }
  } else {
    absolute_position_ = loop_->X_hat(0, 0);
  }

  // switch for dealing with various zeroing states.
  switch (state_) {
    case UNINITIALIZED:
      LOG(DEBUG, "status_: UNINITIALIZED\n");
      last_calibration_sensor_ = -1;
      if (good_position) {
        // Reset the zeroing goal.
        zeroing_position_ = absolute_position_;
        // Clear the observer state.
        loop_->X_hat << absolute_position_, 0.0;
        // Only progress if we are enabled.
        if (::aos::robot_state->enabled) {
          if (HallEffect() != -1) {
            state_ = MOVING_OFF;
          } else {
            state_ = ZEROING;
          }
        } else {
          loop_->R << absolute_position_, 0.0;
        }
      }
      break;
    case MOVING_OFF:
       LOG(DEBUG, "status_: MOVING_OFF\n");
      // Move off the hall effect sensor.
      if (!::aos::robot_state->enabled) {
        // Start over if disabled.
        state_ = UNINITIALIZED;
      } else if (good_position && (HallEffect() == -1)) {
        // We are now off the sensor.  Time to zero now.
        state_ = ZEROING;
      } else {
        // Slowly creep off the sensor.
        zeroing_position_ += zeroing_speed_ * dt;
        loop_->R << zeroing_position_, zeroing_speed_;
        break;
      }
    case ZEROING:
      LOG(DEBUG, "status_: ZEROING\n");
      if (!::aos::robot_state->enabled) {
        // Start over if disabled.
        state_ = UNINITIALIZED;
      } else if (good_position && (HallEffect() != -1)) {
        state_ = READY;
        // Verify that the calibration number is between the last off position
        // and the current on position.  If this is not true, move off and try
        // again.
        if (WhichHallEffect() == -1) {
          LOG(ERROR, "Got a bogus calibration number.  Trying again.\n");
          LOG(ERROR,
              "Last off position was %f, current is %f,\n",
              last_off_position_, current_position_);
          state_ = MOVING_OFF;
        } else {
          // Save the zero, and then offset the observer to deal with the
          // phantom step change.
          const double old_zero_offset = zero_offset_;
          zero_offset_ = calibration_[WhichHallEffect()];
          loop_->X_hat(0, 0) += old_zero_offset - zero_offset_;
          loop_->Y(0, 0) += old_zero_offset - zero_offset_;
          last_calibration_sensor_ = WhichHallEffect();
        }
      } else {
        // Slowly creep towards the sensor.
        zeroing_position_ -= zeroing_speed_ * dt;
        loop_->R << zeroing_position_, -zeroing_speed_;
      }
      break;

    case READY:
      {
        LOG(DEBUG, "status_: READY\n");
        break;
      }

    case ESTOP:
      LOG(WARNING, "have already given up\n");
      return;
  }

  if (state_ == MOVING_OFF || state_ == ZEROING) {
    LimitZeroingGoal();
  }

  if (good_position) {
    LOG(DEBUG, 
        "calibration sensor: %d zero_offset: %f absolute_position: %f\n",
        last_calibration_sensor_, zero_offset_, absolute_position_);
  }

}  // UpdateZeros

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_HALL_EFFECT_INL_H_
