#include "frc971/control_loops/drivetrain/polydrivetrain.h"

#include "aos/common/logging/logging.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"

#include "aos/common/messages/robot_state.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

PolyDrivetrain::PolyDrivetrain(const DrivetrainConfig &dt_config,
                               StateFeedbackLoop<7, 2, 3> *kf)
    : kf_(kf),
      U_Poly_((Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
               /*[*/ -1, 0 /*]*/,
               /*[*/ 0, 1 /*]*/,
               /*[*/ 0, -1 /*]]*/).finished(),
              (Eigen::Matrix<double, 4, 1>() << /*[[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]]*/).finished(),
              (Eigen::Matrix<double, 2, 4>() << /*[[*/ 12, 12, -12, -12 /*]*/,
               /*[*/ -12, 12, 12, -12 /*]*/).finished()),
      loop_(new StateFeedbackLoop<2, 2, 2>(dt_config.make_v_drivetrain_loop())),
      ttrust_(1.1),
      wheel_(0.0),
      throttle_(0.0),
      quickturn_(false),
      left_gear_(dt_config.default_high_gear ? Gear::HIGH : Gear::LOW),
      right_gear_(dt_config.default_high_gear ? Gear::HIGH : Gear::LOW),
      counter_(0),
      dt_config_(dt_config) {
  last_position_.Zero();
  position_.Zero();
}

double PolyDrivetrain::MotorSpeed(
    const constants::ShifterHallEffect &hall_effect, double shifter_position,
    double velocity, Gear gear) {
  const double high_gear_speed =
      velocity / dt_config_.high_gear_ratio / dt_config_.wheel_radius;
  const double low_gear_speed =
      velocity / dt_config_.low_gear_ratio / dt_config_.wheel_radius;

  if (shifter_position < hall_effect.clear_low) {
    // We're in low gear, so return speed for that gear.
    return low_gear_speed;
  } else if (shifter_position > hall_effect.clear_high) {
    // We're in high gear, so return speed for that gear.
    return high_gear_speed;
  }

  // Not in gear, so speed-match to destination gear.
  switch (gear) {
    case Gear::HIGH:
    case Gear::SHIFTING_UP:
      return high_gear_speed;
    case Gear::LOW:
    case Gear::SHIFTING_DOWN:
    default:
      return low_gear_speed;
      break;
  }
}

Gear PolyDrivetrain::UpdateSingleGear(Gear requested_gear, Gear current_gear) {
  const Gear shift_up =
      (dt_config_.shifter_type == ShifterType::HALL_EFFECT_SHIFTER)
          ? Gear::SHIFTING_UP
          : Gear::HIGH;
  const Gear shift_down =
      (dt_config_.shifter_type == ShifterType::HALL_EFFECT_SHIFTER)
          ? Gear::SHIFTING_DOWN
          : Gear::LOW;
  if (current_gear != requested_gear) {
    if (IsInGear(current_gear)) {
      if (requested_gear == Gear::HIGH) {
        if (current_gear != Gear::HIGH) {
          current_gear = shift_up;
        }
      } else {
        if (current_gear != Gear::LOW) {
          current_gear = shift_down;
        }
      }
    } else {
      if (requested_gear == Gear::HIGH && current_gear == Gear::SHIFTING_DOWN) {
        current_gear = Gear::SHIFTING_UP;
      } else if (requested_gear == Gear::LOW &&
                 current_gear == Gear::SHIFTING_UP) {
        current_gear = Gear::SHIFTING_DOWN;
      }
    }
  }
  return current_gear;
}

void PolyDrivetrain::SetGoal(
    const ::frc971::control_loops::DrivetrainQueue::Goal &goal) {
  const double wheel = goal.steering;
  const double throttle = goal.throttle;
  const bool quickturn = goal.quickturn;
  const bool highgear = goal.highgear;

  // Apply a sin function that's scaled to make it feel better.
  const double angular_range = M_PI_2 * dt_config_.wheel_non_linearity;

  wheel_ = sin(angular_range * wheel) / sin(angular_range);
  wheel_ = sin(angular_range * wheel_) / sin(angular_range);
  wheel_ = 2.0 * wheel - wheel_;
  quickturn_ = quickturn;

  if (!quickturn_) {
    wheel_ *= dt_config_.quickturn_wheel_multiplier;
  }

  static const double kThrottleDeadband = 0.05;
  if (::std::abs(throttle) < kThrottleDeadband) {
    throttle_ = 0;
  } else {
    throttle_ = copysign(
        (::std::abs(throttle) - kThrottleDeadband) / (1.0 - kThrottleDeadband),
        throttle);
  }

  Gear requested_gear = highgear ? Gear::HIGH : Gear::LOW;

  left_gear_ = PolyDrivetrain::UpdateSingleGear(requested_gear, left_gear_);
  right_gear_ = PolyDrivetrain::UpdateSingleGear(requested_gear, right_gear_);
}

void PolyDrivetrain::SetPosition(
    const ::frc971::control_loops::DrivetrainQueue::Position *position,
    Gear left_gear, Gear right_gear) {
  left_gear_ = left_gear;
  right_gear_ = right_gear;
  last_position_ = position_;
  position_ = *position;
}

double PolyDrivetrain::FilterVelocity(double throttle) const {
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
  if (dt_config_.loop_type == LoopType::CLOSED_LOOP) {
    loop_->mutable_X_hat()(0, 0) = kf_->X_hat()(1, 0);
    loop_->mutable_X_hat()(1, 0) = kf_->X_hat()(3, 0);
  }

  // TODO(austin): Observer for the current velocity instead of difference
  // calculations.
  ++counter_;

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
    goal_left_velocity_ = left_velocity;
    goal_right_velocity_ = right_velocity;

    // Integrate velocity to get the position.
    // This position is used to get integral control.
    loop_->mutable_R() << left_velocity, right_velocity;

    if (!quickturn_) {
      // K * R = w
      Eigen::Matrix<double, 1, 2> equality_k;
      equality_k << 1 + sign_svel, -(1 - sign_svel);
      const double equality_w = 0.0;

      // Construct a constraint on R by manipulating the constraint on U
      ::aos::controls::HVPolytope<2, 4, 4> R_poly_hv(
          U_Poly_.static_H() * (loop_->K() + FF),
          U_Poly_.static_k() + U_Poly_.static_H() * loop_->K() * loop_->X_hat(),
          (loop_->K() + FF).inverse() *
              ::aos::controls::ShiftPoints<2, 4>(U_Poly_.StaticVertices(),
                                                 loop_->K() * loop_->X_hat()));

      // Limit R back inside the box.
      loop_->mutable_R() =
          CoerceGoal(R_poly_hv, equality_k, equality_w, loop_->R());
    }

    const Eigen::Matrix<double, 2, 1> FF_volts = FF * loop_->R();
    const Eigen::Matrix<double, 2, 1> U_ideal =
        loop_->K() * (loop_->R() - loop_->X_hat()) + FF_volts;

    for (int i = 0; i < 2; i++) {
      loop_->mutable_U()[i] = ::aos::Clip(U_ideal[i], -12, 12);
    }

    if (dt_config_.loop_type == LoopType::OPEN_LOOP) {
      loop_->mutable_X_hat() =
          loop_->A() * loop_->X_hat() + loop_->B() * loop_->U();
    }
  } else {
    const double current_left_velocity =
        (position_.left_encoder - last_position_.left_encoder) / dt_config_.dt;
    const double current_right_velocity =
        (position_.right_encoder - last_position_.right_encoder) /
        dt_config_.dt;
    const double left_motor_speed =
        MotorSpeed(dt_config_.left_drive, position_.left_shifter_position,
                   current_left_velocity, left_gear_);
    const double right_motor_speed =
        MotorSpeed(dt_config_.right_drive, position_.right_shifter_position,
                   current_right_velocity, right_gear_);

    {
      CIMLogging logging;

      // Reset the CIM model to the current conditions to be ready for when we
      // shift.
      logging.left_in_gear = IsInGear(left_gear_);
      logging.left_motor_speed = left_motor_speed;
      logging.left_velocity = current_left_velocity;

      logging.right_in_gear = IsInGear(right_gear_);
      logging.right_motor_speed = right_motor_speed;
      logging.right_velocity = current_right_velocity;

      LOG_STRUCT(DEBUG, "currently", logging);
    }
    goal_left_velocity_ = current_left_velocity;
    goal_right_velocity_ = current_right_velocity;

    // Any motor is not in gear. Speed match.
    ::Eigen::Matrix<double, 1, 1> R_left;
    ::Eigen::Matrix<double, 1, 1> R_right;
    R_left(0, 0) = left_motor_speed;
    R_right(0, 0) = right_motor_speed;

    const double wiggle =
        (static_cast<double>((counter_ % 30) / 15) - 0.5) * 8.0;

    loop_->mutable_U(0, 0) = ::aos::Clip(
        (R_left / dt_config_.v)(0, 0) + (IsInGear(left_gear_) ? 0 : wiggle),
        -12.0, 12.0);
    loop_->mutable_U(1, 0) = ::aos::Clip(
        (R_right / dt_config_.v)(0, 0) + (IsInGear(right_gear_) ? 0 : wiggle),
        -12.0, 12.0);
    loop_->mutable_U() *= 12.0 / ::aos::robot_state->voltage_battery;
  }
}

void PolyDrivetrain::SetOutput(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  if (output != NULL) {
    output->left_voltage = loop_->U(0, 0);
    output->right_voltage = loop_->U(1, 0);
    output->left_high = MaybeHigh(left_gear_);
    output->right_high = MaybeHigh(right_gear_);
  }
}

void PolyDrivetrain::PopulateStatus(
    ::frc971::control_loops::DrivetrainQueue::Status *status) {
  status->left_velocity_goal = goal_left_velocity_;
  status->right_velocity_goal = goal_right_velocity_;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
