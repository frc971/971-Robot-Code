#include "frc971/control_loops/angle_adjust/angle_adjust.h"
#include "frc971/control_loops/hall_effect_loop.h"
#include "frc971/control_loops/hall_effect_loop-inl.h"

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"

namespace frc971 {
namespace control_loops {

AngleAdjustMotor::AngleAdjustMotor(
    control_loops::AngleAdjustLoop *my_angle_adjust)
    : aos::control_loops::ControlLoop<control_loops::AngleAdjustLoop>(
        my_angle_adjust),
    hall_effect_(new StateFeedbackLoop<2, 1, 1>(MakeAngleAdjustLoop()), true),
    error_count_(0),
    time_(0.0) {
  if (testing) {
    hall_effect_.loop_->StartDataFile("angle_adjust.csv");
  }
}

bool AngleAdjustMotor::FetchConstants() {
  if (!constants::angle_adjust_horizontal_lower_limit(
          &horizontal_lower_limit_)) {
    LOG(ERROR, "Failed to fetch the horizontal lower limit constant.\n");
    return false;
  }
  if (!constants::angle_adjust_horizontal_upper_limit(
          &horizontal_upper_limit_)) {
    LOG(ERROR, "Failed to fetch the horizontal upper limit constant.\n");
    return false;
  }
  if (!constants::angle_adjust_horizontal_hall_effect_stop_angle(
          &horizontal_hall_effect_stop_angle_)) {
    LOG(ERROR, "Failed to fetch the hall effect stop angle constants.\n");
    return false;
  }
  if (!constants::angle_adjust_horizontal_zeroing_speed(
          &horizontal_zeroing_speed_)) {
    LOG(ERROR, "Failed to fetch the horizontal zeroing speed constant.\n");
    return false;
  }

  return true;
}

double AngleAdjustMotor::ClipGoal(double goal) const {
  return std::min(horizontal_upper_limit_,
                  std::max(horizontal_lower_limit_, goal));
}

double AngleAdjustMotor::LimitVoltage(double absolute_position,
                                double voltage) const {
  if (hall_effect_.state_ == HallEffectLoop<2>::READY) {
    if (absolute_position >= horizontal_upper_limit_) {
      voltage = std::min(0.0, voltage);
    }
    if (absolute_position <= horizontal_lower_limit_) {
      voltage = std::max(0.0, voltage);
    }
  }

  double limit = (hall_effect_.state_ == HallEffectLoop<2>::READY) ? 12.0 : 5.0;
  // TODO(aschuh): Remove this line when we are done testing.
  //limit = std::min(0.3, limit);
  voltage = std::min(limit, voltage);
  voltage = std::max(-limit, voltage);
  return voltage;
}

// Positive angle is up, and positive power is up.
void AngleAdjustMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::AngleAdjustLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * /*status*/) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->voltage = 0;
  }

  // Cache the constants to avoid error handling down below.
  if (!FetchConstants()) {
    LOG(WARNING, "Failed to fetch constants.\n");
    return;
  }

  // Uninitialize the bot if too many cycles pass without an encoder.
  if (position == NULL) {
    LOG(WARNING, "no new pos given\n");
    error_count_++;
  } else {
    error_count_ = 0;
  }
  if (error_count_ >= 4) {
    LOG(WARNING, "err_count is %d so forcing a re-zero\n", error_count_);
    hall_effect_.state_ = HallEffectLoop<2>::UNINITIALIZED;
  }

  double absolute_position = hall_effect_.loop_->X_hat(0, 0);
  // Compute the absolute position of the angle adjust.
  if (position) {
    hall_effect_sensors_[0] = position->bottom_hall_effect;
    hall_effect_sensors_[1] = position->middle_hall_effect;
    calibration_values_[0] = position->bottom_calibration;
    calibration_values_[1] = position->middle_calibration;
    absolute_position = position->before_angle;
  }

  hall_effect_.UpdateZeros(horizontal_hall_effect_stop_angle_,
              hall_effect_sensors_,
              calibration_values_,
              horizontal_zeroing_speed_,
              absolute_position,
              position != NULL);

  if (hall_effect_.state_ == HallEffectLoop<2>::READY) {
    const double limited_goal = ClipGoal(goal->goal);
    hall_effect_.loop_->R << limited_goal, 0.0;
  }

  // Update the observer.
  hall_effect_.loop_->Update(position != NULL, output == NULL);

  if (position) {
    LOG(DEBUG, "pos=%f bottom_hall: %s middle_hall: %s\n",
        position->before_angle,
        position->bottom_hall_effect ? "true" : "false",
        position->middle_hall_effect ? "true" : "false");
  }

  if (hall_effect_.state_ == HallEffectLoop<2>::READY) {
    LOG(DEBUG, "calibrated with: %s hall effect\n",
        hall_effect_.last_calibration_sensor_ ? "bottom" : "middle");
  }

  if (output) {
    output->voltage = LimitVoltage(hall_effect_.absolute_position_,
                                   hall_effect_.loop_->U(0, 0));
  }

  if (testing) {
    hall_effect_.loop_->RecordDatum("angle_adjust.csv", time_);
  }
  time_ += dt;
} // RunIteration

}  // namespace control_loops
}  // namespace frc971
