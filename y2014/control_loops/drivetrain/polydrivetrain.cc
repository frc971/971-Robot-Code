#include "y2014/control_loops/drivetrain/polydrivetrain.h"

#include "aos/common/logging/logging.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"

#include "aos/common/messages/robot_state.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#define HAVE_SHIFTERS 1

namespace y2014 {
namespace control_loops {
namespace drivetrain {

using ::y2014::control_loops::GearLogging;
using ::y2014::control_loops::CIMLogging;
using ::frc971::control_loops::CoerceGoal;

PolyDrivetrain::PolyDrivetrain(StateFeedbackLoop<7, 2, 3> *kf)
    : kf_(kf),
      U_Poly_((Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
               /*[*/ -1, 0 /*]*/,
               /*[*/ 0, 1 /*]*/,
               /*[*/ 0, -1 /*]]*/).finished(),
              (Eigen::Matrix<double, 4, 1>() << /*[[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]]*/).finished()),
      loop_(new StateFeedbackLoop<2, 2, 2>(
          constants::GetValues().make_v_drivetrain_loop())),
      ttrust_(1.1),
      wheel_(0.0),
      throttle_(0.0),
      quickturn_(false),
      stale_count_(0),
      position_time_delta_(kDt),
      left_gear_(LOW),
      right_gear_(LOW),
      counter_(0) {
  last_position_.Zero();
  position_.Zero();
}

double PolyDrivetrain::MotorSpeed(
    const constants::ShifterHallEffect &hall_effect, double shifter_position,
    double velocity) {
  const double avg_hall_effect =
      (hall_effect.clear_high + hall_effect.clear_low) / 2.0;

  if (shifter_position > avg_hall_effect) {
    return velocity / constants::GetValues().high_gear_ratio / kWheelRadius;
  } else {
    return velocity / constants::GetValues().low_gear_ratio / kWheelRadius;
  }
}

PolyDrivetrain::Gear PolyDrivetrain::UpdateSingleGear(
    Gear requested_gear, Gear current_gear) {
  const Gear shift_up =
      constants::GetValues().clutch_transmission ? HIGH : SHIFTING_UP;
  const Gear shift_down =
      constants::GetValues().clutch_transmission ? LOW : SHIFTING_DOWN;

  if (current_gear != requested_gear) {
    if (IsInGear(current_gear)) {
      if (requested_gear == HIGH) {
        if (current_gear != HIGH) {
          current_gear = shift_up;
        }
      } else {
        if (current_gear != LOW) {
          current_gear = shift_down;
        }
      }
    } else {
      if (requested_gear == HIGH && current_gear == SHIFTING_DOWN) {
        current_gear = SHIFTING_UP;
      } else if (requested_gear == LOW && current_gear == SHIFTING_UP) {
        current_gear = SHIFTING_DOWN;
      }
    }
  }
  return current_gear;
}

void PolyDrivetrain::UpdateGears(Gear requested_gear) {
  left_gear_ = UpdateSingleGear(requested_gear, left_gear_);
  right_gear_ = UpdateSingleGear(requested_gear, right_gear_);
}

void PolyDrivetrain::SetGoal(double wheel, double throttle, bool quickturn,
                             bool highgear) {
  const double kWheelNonLinearity = 0.3;
  // Apply a sin function that's scaled to make it feel better.
  const double angular_range = M_PI_2 * kWheelNonLinearity;

  wheel_ = sin(angular_range * wheel) / sin(angular_range);
  wheel_ = sin(angular_range * wheel_) / sin(angular_range);
  quickturn_ = quickturn;

  static const double kThrottleDeadband = 0.05;
  if (::std::abs(throttle) < kThrottleDeadband) {
    throttle_ = 0;
  } else {
    throttle_ = copysign(
        (::std::abs(throttle) - kThrottleDeadband) / (1.0 - kThrottleDeadband),
        throttle);
  }

  UpdateGears(highgear ? HIGH : LOW);
}

void PolyDrivetrain::SetPosition(
    const ::y2014::control_loops::DrivetrainQueue::Position *position) {
  if (position == NULL) {
    ++stale_count_;
  } else {
    last_position_ = position_;
    position_ = *position;
    position_time_delta_ = (stale_count_ + 1) * kDt;
    stale_count_ = 0;
  }

  if (position) {
    const auto &values = constants::GetValues();
    GearLogging gear_logging;
    // Switch to the correct controller.
    const double left_middle_shifter_position =
        (values.left_drive.clear_high + values.left_drive.clear_low) / 2.0;
    const double right_middle_shifter_position =
        (values.right_drive.clear_high + values.right_drive.clear_low) / 2.0;

    if (position->left_shifter_position < left_middle_shifter_position ||
        left_gear_ == LOW) {
      if (position->right_shifter_position < right_middle_shifter_position ||
          right_gear_ == LOW) {
        gear_logging.left_loop_high = false;
        gear_logging.right_loop_high = false;
        loop_->set_controller_index(gear_logging.controller_index = 0);
      } else {
        gear_logging.left_loop_high = false;
        gear_logging.right_loop_high = true;
        loop_->set_controller_index(gear_logging.controller_index = 1);
      }
    } else {
      if (position->right_shifter_position < right_middle_shifter_position ||
          right_gear_ == LOW) {
        gear_logging.left_loop_high = true;
        gear_logging.right_loop_high = false;
        loop_->set_controller_index(gear_logging.controller_index = 2);
      } else {
        gear_logging.left_loop_high = true;
        gear_logging.right_loop_high = true;
        loop_->set_controller_index(gear_logging.controller_index = 3);
      }
    }

    if (position->left_shifter_position > values.left_drive.clear_high &&
        left_gear_ == SHIFTING_UP) {
      left_gear_ = HIGH;
    }
    if (position->left_shifter_position < values.left_drive.clear_low &&
        left_gear_ == SHIFTING_DOWN) {
      left_gear_ = LOW;
    }
    if (position->right_shifter_position > values.right_drive.clear_high &&
        right_gear_ == SHIFTING_UP) {
      right_gear_ = HIGH;
    }
    if (position->right_shifter_position < values.right_drive.clear_low &&
        right_gear_ == SHIFTING_DOWN) {
      right_gear_ = LOW;
    }

    gear_logging.left_state = left_gear_;
    gear_logging.right_state = right_gear_;
    LOG_STRUCT(DEBUG, "state", gear_logging);
  }
}

double PolyDrivetrain::FilterVelocity(double throttle) {
  const Eigen::Matrix<double, 2, 2> FF =
      loop_->B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<double, 2, 2> FF_high =
      loop_->controller(kHighGearController).plant.B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() -
       loop_->controller(kHighGearController).plant.A());

  ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
  const double high_min_FF_sum = FF_high.col(0).sum();

  const double adjusted_ff_voltage =
      ::aos::Clip(throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
  return (adjusted_ff_voltage +
          ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) /
              2.0) /
         (ttrust_ * min_K_sum + min_FF_sum);
}

double PolyDrivetrain::MaxVelocity() {
  const Eigen::Matrix<double, 2, 2> FF =
      loop_->B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<double, 2, 2> FF_high =
      loop_->controller(kHighGearController).plant.B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() -
       loop_->controller(kHighGearController).plant.A());

  ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  // const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
  const double high_min_FF_sum = FF_high.col(0).sum();

  const double adjusted_ff_voltage =
      ::aos::Clip(12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
  return adjusted_ff_voltage / min_FF_sum;
}

void PolyDrivetrain::Update() {
  loop_->mutable_X_hat()(0, 0) = kf_->X_hat()(1, 0);
  loop_->mutable_X_hat()(1, 0) = kf_->X_hat()(3, 0);

  const auto &values = constants::GetValues();
  // TODO(austin): Observer for the current velocity instead of difference
  // calculations.
  ++counter_;
  const double current_left_velocity =
      (position_.left_encoder - last_position_.left_encoder) /
      position_time_delta_;
  const double current_right_velocity =
      (position_.right_encoder - last_position_.right_encoder) /
      position_time_delta_;
  const double left_motor_speed =
      MotorSpeed(values.left_drive, position_.left_shifter_position,
                 current_left_velocity);
  const double right_motor_speed =
      MotorSpeed(values.right_drive, position_.right_shifter_position,
                 current_right_velocity);

  {
    CIMLogging logging;

    // Reset the CIM model to the current conditions to be ready for when we
    // shift.
    if (IsInGear(left_gear_)) {
      logging.left_in_gear = true;
    } else {
      logging.left_in_gear = false;
    }
    logging.left_motor_speed = left_motor_speed;
    logging.left_velocity = current_left_velocity;
    if (IsInGear(right_gear_)) {
      logging.right_in_gear = true;
    } else {
      logging.right_in_gear = false;
    }
    logging.right_motor_speed = right_motor_speed;
    logging.right_velocity = current_right_velocity;

    LOG_STRUCT(DEBUG, "currently", logging);
  }

  if (IsInGear(left_gear_) && IsInGear(right_gear_)) {
    // FF * X = U (steady state)
    const Eigen::Matrix<double, 2, 2> FF =
        loop_->B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

    // Invert the plant to figure out how the velocity filter would have to
    // work
    // out in order to filter out the forwards negative inertia.
    // This math assumes that the left and right power and velocity are
    // equals,
    // and that the plant is the same on the left and right.
    const double fvel = FilterVelocity(throttle_);

    const double sign_svel = wheel_ * ((fvel > 0.0) ? 1.0 : -1.0);
    double steering_velocity;
    if (quickturn_) {
      steering_velocity = wheel_ * MaxVelocity();
    } else {
      steering_velocity = ::std::abs(fvel) * wheel_;
    }
    const double left_velocity = fvel - steering_velocity;
    const double right_velocity = fvel + steering_velocity;

    // Integrate velocity to get the position.
    // This position is used to get integral control.
    loop_->mutable_R() << left_velocity, right_velocity;

    if (!quickturn_) {
      // K * R = w
      Eigen::Matrix<double, 1, 2> equality_k;
      equality_k << 1 + sign_svel, -(1 - sign_svel);
      const double equality_w = 0.0;

      // Construct a constraint on R by manipulating the constraint on U
      ::aos::controls::HPolytope<2> R_poly = ::aos::controls::HPolytope<2>(
          U_Poly_.H() * (loop_->K() + FF),
          U_Poly_.k() + U_Poly_.H() * loop_->K() * loop_->X_hat());

      // Limit R back inside the box.
      loop_->mutable_R() =
          CoerceGoal(R_poly, equality_k, equality_w, loop_->R());
    }

    const Eigen::Matrix<double, 2, 1> FF_volts = FF * loop_->R();
    const Eigen::Matrix<double, 2, 1> U_ideal =
        loop_->K() * (loop_->R() - loop_->X_hat()) + FF_volts;

    for (int i = 0; i < 2; i++) {
      loop_->mutable_U()[i] = ::aos::Clip(U_ideal[i], -12, 12);
    }
  } else {
    // Any motor is not in gear.  Speed match.
    ::Eigen::Matrix<double, 1, 1> R_left;
    ::Eigen::Matrix<double, 1, 1> R_right;
    R_left(0, 0) = left_motor_speed;
    R_right(0, 0) = right_motor_speed;

    const double wiggle =
        (static_cast<double>((counter_ % 20) / 10) - 0.5) * 5.0;

    loop_->mutable_U(0, 0) = ::aos::Clip(
        (R_left / Kv)(0, 0) + (IsInGear(left_gear_) ? 0 : wiggle), -12.0, 12.0);
    loop_->mutable_U(1, 0) =
        ::aos::Clip((R_right / Kv)(0, 0) + (IsInGear(right_gear_) ? 0 : wiggle),
                    -12.0, 12.0);
    loop_->mutable_U() *= 12.0 / ::aos::robot_state->voltage_battery;
  }
}

void PolyDrivetrain::SendMotors(
    ::y2014::control_loops::DrivetrainQueue::Output *output) {
  if (output != NULL) {
    output->left_voltage = loop_->U(0, 0);
    output->right_voltage = loop_->U(1, 0);
    output->left_high = left_gear_ == HIGH || left_gear_ == SHIFTING_UP;
    output->right_high = right_gear_ == HIGH || right_gear_ == SHIFTING_UP;
  }
}

constexpr double PolyDrivetrain::kStallTorque;
constexpr double PolyDrivetrain::kStallCurrent;
constexpr double PolyDrivetrain::kFreeSpeed;
constexpr double PolyDrivetrain::kFreeCurrent;
constexpr double PolyDrivetrain::kWheelRadius;
constexpr double PolyDrivetrain::kR;
constexpr double PolyDrivetrain::Kv;
constexpr double PolyDrivetrain::Kt;

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014
