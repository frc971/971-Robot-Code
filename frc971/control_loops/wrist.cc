#include "frc971/control_loops/wrist.h"

#include <stdio.h>

#include <algorithm>

#include "aos/aos_core.h"

#include "aos/common/messages/RobotState.q.h"
#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/wrist_motor_plant.h"

namespace frc971 {
namespace control_loops {

static const double MIN_POS = -0.80;
static const double MAX_POS = 1.77;
static const double positive_deadband_power = 0.15 * 12.0;
static const double negative_deadband_power = 0.09 * 12.0;
// Final units is radians/iteration.
static const double MAX_SPEED = 19300.0/*free RPM*/ * 12.0/*pinions*/ / 60.0 * 14.0 / 50.0 * 14.0 / 48.0 / 60.0/*60 sec/min*/ / 100.0/*100 cycles/sec*/ * (2.0 * M_PI)/*rotations to radians*/;
// The minimum magnitude for X_hat[1] to be sure of the direction it's moving so
// that the code can get a zero. X_hat[1] is the angular velocity of the wrist
// predicted by the observer, and it can be wrong when the wrist is just
// switching direction and/or it is moving very slowly in/around the victor
// deadband. Using this threshold instead of just 0 helps avoid falsely zeroing
// on the back edge of the magnet instead of the front one.
static const double kX_hatThreshold = 0.2;
// The maximum amount that the calibration value can be off for it to be
// accepted.
// MAX_SPEED / 10 is the amount it would be off if it was read 1ms earlier than
// the actual encoder position, which is a lot more than it should ever be.
static const double kMaxCalibrationError = MAX_SPEED / 10;

WristMotor::WristMotor(control_loops::WristLoop *my_wrist)
    : aos::control_loops::ControlLoop<control_loops::WristLoop>(my_wrist),
      loop_(new StateFeedbackLoop<2, 1, 1>(MakeWristLoop())),
      stop_(false),
      error_count_(0),
      zeroed_(false),
      zero_offset_(0.0) { 
}

// down is positive
void WristMotor::RunIteration(
    const ::aos::control_loops::Goal *goal,
    const control_loops::WristLoop::Position *position,
    ::aos::control_loops::Output *output,
    ::aos::control_loops::Status * /*status*/) {

  if (output) {
    output->voltage = 0;
  }

  if (stop_) {
    LOG(WARNING, "have already given up\n");
    return;
  }

  if (position == NULL) {
    LOG(WARNING, "no new pos given\n");
    error_count_++;
  } else {
    error_count_ = 0;
  }

  double horizontal;
  if (!constants::horizontal_offset(&horizontal)) {
    LOG(ERROR, "Failed to fetch the horizontal offset constant.\n");
    return;
  }

  static bool good_zeroed = false;
  static bool first = true;
  if (error_count_ >= 4) {
    LOG(WARNING, "err_count is %d so forcing a re-zero\n", error_count_);
    first = true;
    zeroed_ = false;
    good_zeroed = false;
  }

  const double limited_goal = std::min(MAX_POS, std::max(MIN_POS, goal->goal));

  static double zero_goal;
  static bool old_cal = false;
  static double old_pos = -5000000;
  bool bad_encoder = false;
  if (!first) {
    if (old_cal != position->hall_effect) {
      LOG(INFO, "old=%s X_hat=%f\n", old_cal ? "true" : "false", loop_->X_hat[1]);
      // Don't want to miss the first one if it starts really close to the edge
      // of the magnet so isn't moving very fast when it passes.
      const double X_hatThreshold = zeroed_ ? kX_hatThreshold : 0;
      if ((old_cal && loop_->X_hat[1] < -X_hatThreshold) ||
          (!old_cal && loop_->X_hat[1] > X_hatThreshold)) {
        LOG(INFO, "zeroing at %f (pos=%f, old_pos=%f) (used to be %f)\n",
            position->calibration, position->pos, old_pos, zero_offset_);
        // If ((old_pos - 0.02) <= position->calibration <= position->pos) or
        // the other way around. Checks if position->calibration is between
        // old_pos and position->pos if the wrist is moving either way, with a
        // slight tweak to
        // old_pos in case it was triggered while the sensor packet was being
        // sent or something.
        if (!((old_pos - kMaxCalibrationError <= position->calibration &&
               position->calibration <= position->pos) ||
              (old_pos + kMaxCalibrationError >= position->calibration &&
               position->calibration >= position->pos))) {
          if (!good_zeroed) {
            LOG(WARNING, "using encoder pos because "
                "calibration sensor pos doesn't make sense\n");
            zero_offset_ = position->pos;
          } else {
            LOG(INFO, "already had a good zero, "
                "so not using inaccurate zero value\n");
          }
        } else {
          good_zeroed = true;
          zero_offset_ = position->calibration;
        }
        zeroed_ = true;
      } else {
        LOG(INFO, "hit back edge at %f\n", position->calibration);
      }
      old_cal = position->hall_effect;
    }
    if (std::abs(position->pos - old_pos) > MAX_SPEED * 1.5) {
      bad_encoder = true;
      LOG(WARNING, "encoder value changed by %f which is more than MAX_SPEED(%f)\n",
          std::abs(position->pos - old_pos), MAX_SPEED);
    }
  } // !first

  old_pos = position->pos;
  const double absolute_position = position->pos - zero_offset_ - horizontal;
  if (first) {
    first = false;
    old_cal = position->hall_effect;
    zero_goal = absolute_position;
  }

  loop_->Y << absolute_position;
  if (!zeroed_) {
    loop_->R << zero_goal, 0.0;
    if (aos::robot_state->enabled) {
      if (!position->hall_effect) {
        zero_goal += 0.010;
      } else {
        zero_goal -= 0.010;
      }
    }
  } else {
    loop_->R << limited_goal, 0.0;
  }
  loop_->Update(!bad_encoder, bad_encoder || output == NULL);
  double output_voltage = loop_->U[0];
  //LOG(DEBUG, "fancy math gave %f\n", status->pwm);

  if (output_voltage > 0) {
    output_voltage += positive_deadband_power;
  }
  if (output_voltage < 0) {
    output_voltage -= negative_deadband_power;
  }

  if (zeroed_) {
    if (absolute_position >= MAX_POS) {
      output_voltage = std::min(0.0, output_voltage);
    }
    if (absolute_position <= MIN_POS) {
      output_voltage = std::max(0.0, output_voltage);
    }
  }

  double limit = zeroed_ ? 1.0 : 0.5;
  //limit = std::min(0.3, limit);
  output_voltage = std::min(limit, output_voltage);
  output_voltage = std::max(-limit, output_voltage);
  LOG(DEBUG, "pos=%f zero=%f horizontal=%f currently %f hall: %s\n",
      position->pos, zero_offset_, horizontal, absolute_position,
      position->hall_effect ? "true" : "false");
  if (output) {
    output->voltage = output_voltage;
  }
}

}  // namespace control_loops
}  // namespace frc971
