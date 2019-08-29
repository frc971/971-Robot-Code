#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_

#include "aos/controls/polytope.h"

#include "aos/commonmath.h"
#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/gear.h"
#ifdef __linux__
#include "aos/logging/logging.h"
#include "aos/robot_state/robot_state_generated.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#else
#include "frc971/control_loops/drivetrain/drivetrain_goal_float_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_float_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_float_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_float_generated.h"
#endif  // __linux__
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

template <typename Scalar = double>
class PolyDrivetrain {
 public:
  PolyDrivetrain(const DrivetrainConfig<Scalar> &dt_config,
                 StateFeedbackLoop<7, 2, 4, Scalar> *kf);

  int controller_index() const { return loop_->index(); }

  // Computes the speed of the motor given the hall effect position and the
  // speed of the robot.
  Scalar MotorSpeed(const constants::ShifterHallEffect &hall_effect,
                    Scalar shifter_position, Scalar velocity, Gear gear);

  void SetGoal(const Scalar wheel, const Scalar throttle, const bool quickturn,
               const bool highgear);

  void SetPosition(
      const ::frc971::control_loops::drivetrain::Position *position,
      Gear left_gear, Gear right_gear);

  Scalar FilterVelocity(Scalar throttle) const;

  Scalar MaxVelocity();

  void Update(Scalar voltage_battery);

  void SetOutput(::frc971::control_loops::drivetrain::OutputT *output);

  flatbuffers::Offset<CIMLogging> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  // Computes the next state of a shifter given the current state and the
  // requested state.
  Gear UpdateSingleGear(Gear requested_gear, Gear current_gear);

  // Returns the current estimated velocity in m/s.
  Scalar velocity() const {
    return (loop_->mutable_X_hat()(0) + loop_->mutable_X_hat()(1)) * kHalf;
  }

 private:
  static constexpr Scalar kZero = static_cast<Scalar>(0.0);
  static constexpr Scalar kHalf = static_cast<Scalar>(0.5);
  static constexpr Scalar kOne = static_cast<Scalar>(1.0);
  static constexpr Scalar kTwo = static_cast<Scalar>(2.0);
  static constexpr Scalar kTwelve = static_cast<Scalar>(12.0);

  StateFeedbackLoop<7, 2, 4, Scalar> *kf_;

  const ::aos::controls::HVPolytope<2, 4, 4, Scalar> U_Poly_;

  ::std::unique_ptr<StateFeedbackLoop<2, 2, 2, Scalar>> loop_;

  const Scalar ttrust_;
  Scalar wheel_;
  Scalar throttle_;
  bool quickturn_;

  Gear left_gear_;
  Gear right_gear_;

  ::frc971::control_loops::drivetrain::PositionT last_position_;
  ::frc971::control_loops::drivetrain::PositionT position_;
  int counter_;
  DrivetrainConfig<Scalar> dt_config_;

  Scalar goal_left_velocity_ = 0.0;
  Scalar goal_right_velocity_ = 0.0;

  // Stored from the last iteration, for logging shifting logic.
  Scalar left_motor_speed_ = 0.0;
  Scalar right_motor_speed_ = 0.0;
  Scalar current_left_velocity_ = 0.0;
  Scalar current_right_velocity_ = 0.0;
};

template <typename Scalar>
PolyDrivetrain<Scalar>::PolyDrivetrain(
    const DrivetrainConfig<Scalar> &dt_config,
    StateFeedbackLoop<7, 2, 4, Scalar> *kf)
    : kf_(kf),
      U_Poly_((Eigen::Matrix<Scalar, 4, 2>() << /*[[*/ 1, 0 /*]*/,
               /*[*/ -1, 0 /*]*/,
               /*[*/ 0, 1 /*]*/,
               /*[*/ 0, -1 /*]]*/)
                  .finished(),
              (Eigen::Matrix<Scalar, 4, 1>() << /*[[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]]*/)
                  .finished(),
              (Eigen::Matrix<Scalar, 2, 4>() << /*[[*/ 12, 12, -12, -12 /*]*/,
               /*[*/ -12, 12, 12, -12 /*]*/)
                  .finished()),
      loop_(new StateFeedbackLoop<2, 2, 2, Scalar>(
          dt_config.make_v_drivetrain_loop())),
      ttrust_(1.1),
      wheel_(0.0),
      throttle_(0.0),
      quickturn_(false),
      left_gear_(dt_config.default_high_gear ? Gear::HIGH : Gear::LOW),
      right_gear_(dt_config.default_high_gear ? Gear::HIGH : Gear::LOW),
      counter_(0),
      dt_config_(dt_config) {}

template <typename Scalar>
Scalar PolyDrivetrain<Scalar>::MotorSpeed(
    const constants::ShifterHallEffect &hall_effect, Scalar shifter_position,
    Scalar velocity, Gear gear) {
  const Scalar high_gear_speed =
      velocity /
      static_cast<Scalar>(dt_config_.high_gear_ratio / dt_config_.wheel_radius);
  const Scalar low_gear_speed =
      velocity /
      static_cast<Scalar>(dt_config_.low_gear_ratio / dt_config_.wheel_radius);

  if (shifter_position < static_cast<Scalar>(hall_effect.clear_low)) {
    // We're in low gear, so return speed for that gear.
    return low_gear_speed;
  } else if (shifter_position > static_cast<Scalar>(hall_effect.clear_high)) {
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

template <typename Scalar>
Gear PolyDrivetrain<Scalar>::UpdateSingleGear(Gear requested_gear,
                                              Gear current_gear) {
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

template <typename Scalar>
void PolyDrivetrain<Scalar>::SetGoal(const Scalar wheel, const Scalar throttle,
                                     const bool quickturn,
                                     const bool highgear) {
  // Apply a sin function that's scaled to make it feel better.
  const Scalar angular_range =
      static_cast<Scalar>(M_PI_2) * dt_config_.wheel_non_linearity;

  wheel_ = sin(angular_range * wheel) / sin(angular_range);
  wheel_ = sin(angular_range * wheel_) / sin(angular_range);
  wheel_ = kTwo * wheel - wheel_;
  quickturn_ = quickturn;

  if (quickturn_) {
    wheel_ *= dt_config_.quickturn_wheel_multiplier;
  } else {
    wheel_ *= dt_config_.wheel_multiplier;
  }

  static constexpr Scalar kThrottleDeadband = static_cast<Scalar>(0.05);
  if (::std::abs(throttle) < kThrottleDeadband) {
    throttle_ = 0;
  } else {
    throttle_ = copysign(
        (::std::abs(throttle) - kThrottleDeadband) / (kOne - kThrottleDeadband),
        throttle);
  }

  Gear requested_gear = highgear ? Gear::HIGH : Gear::LOW;

  left_gear_ = UpdateSingleGear(requested_gear, left_gear_);
  right_gear_ = UpdateSingleGear(requested_gear, right_gear_);
}

template <typename Scalar>
void PolyDrivetrain<Scalar>::SetPosition(
    const ::frc971::control_loops::drivetrain::Position *position,
    Gear left_gear, Gear right_gear) {
  left_gear_ = left_gear;
  right_gear_ = right_gear;
  last_position_ = position_;
  position->UnPackTo(&position_);
}

template <typename Scalar>
Scalar PolyDrivetrain<Scalar>::FilterVelocity(Scalar throttle) const {
  const Eigen::Matrix<Scalar, 2, 2> FF =
      loop_->plant().B().inverse() *
      (Eigen::Matrix<Scalar, 2, 2>::Identity() - loop_->plant().A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<Scalar, 2, 2> FF_high =
      loop_->plant().coefficients(kHighGearController).B.inverse() *
      (Eigen::Matrix<Scalar, 2, 2>::Identity() -
       loop_->plant().coefficients(kHighGearController).A);

  ::Eigen::Matrix<Scalar, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const Scalar min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  const Scalar min_K_sum = loop_->controller().K().col(min_FF_sum_index).sum();
  const Scalar high_min_FF_sum = FF_high.col(0).sum();

  const Scalar adjusted_ff_voltage =
      ::aos::Clip(throttle * kTwelve * min_FF_sum / high_min_FF_sum, -kTwelve, kTwelve);
  return (adjusted_ff_voltage +
          ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) *
              kHalf) /
         (ttrust_ * min_K_sum + min_FF_sum);
}

template <typename Scalar>
Scalar PolyDrivetrain<Scalar>::MaxVelocity() {
  const Eigen::Matrix<Scalar, 2, 2> FF =
      loop_->plant().B().inverse() *
      (Eigen::Matrix<Scalar, 2, 2>::Identity() - loop_->plant().A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<Scalar, 2, 2> FF_high =
      loop_->plant().coefficients(kHighGearController).B.inverse() *
      (Eigen::Matrix<Scalar, 2, 2>::Identity() -
       loop_->plant().coefficients(kHighGearController).A);

  ::Eigen::Matrix<Scalar, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const Scalar min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  // const Scalar min_K_sum = loop_->K().col(min_FF_sum_index).sum();
  const Scalar high_min_FF_sum = FF_high.col(0).sum();

  const Scalar adjusted_ff_voltage =
      ::aos::Clip(kTwelve * min_FF_sum / high_min_FF_sum, -kTwelve, kTwelve);
  return adjusted_ff_voltage / min_FF_sum;
}

template <typename Scalar>
void PolyDrivetrain<Scalar>::Update(Scalar voltage_battery) {
  if (dt_config_.loop_type == LoopType::CLOSED_LOOP) {
    loop_->mutable_X_hat()(0, 0) = kf_->X_hat()(1, 0);
    loop_->mutable_X_hat()(1, 0) = kf_->X_hat()(3, 0);
  }

  // TODO(austin): Observer for the current velocity instead of difference
  // calculations.
  ++counter_;

  if (IsInGear(left_gear_) && IsInGear(right_gear_)) {
    // FF * X = U (steady state)
    const Eigen::Matrix<Scalar, 2, 2> FF =
        loop_->plant().B().inverse() *
        (Eigen::Matrix<Scalar, 2, 2>::Identity() - loop_->plant().A());

    // Invert the plant to figure out how the velocity filter would have to
    // work
    // out in order to filter out the forwards negative inertia.
    // This math assumes that the left and right power and velocity are
    // equals,
    // and that the plant is the same on the left and right.
    const Scalar fvel = FilterVelocity(throttle_);

    const Scalar sign_svel = wheel_ * ((fvel > kZero) ? kOne : -kOne);
    Scalar steering_velocity;
    if (quickturn_) {
      steering_velocity = wheel_ * MaxVelocity();
    } else {
      steering_velocity = ::std::abs(fvel) * wheel_;
    }
    const Scalar left_velocity = fvel - steering_velocity;
    const Scalar right_velocity = fvel + steering_velocity;
    goal_left_velocity_ = left_velocity;
    goal_right_velocity_ = right_velocity;

    // Integrate velocity to get the position.
    // This position is used to get integral control.
    loop_->mutable_R() << left_velocity, right_velocity;

    if (!quickturn_) {
      // K * R = w
      Eigen::Matrix<Scalar, 1, 2> equality_k;
      equality_k << 1 + sign_svel, -(1 - sign_svel);
      const Scalar equality_w = kZero;

      // Construct a constraint on R by manipulating the constraint on U
      ::aos::controls::HVPolytope<2, 4, 4, Scalar> R_poly_hv(
          U_Poly_.static_H() * (loop_->controller().K() + FF),
          U_Poly_.static_k() +
              U_Poly_.static_H() * loop_->controller().K() * loop_->X_hat(),
          (loop_->controller().K() + FF).inverse() *
              ::aos::controls::ShiftPoints<2, 4, Scalar>(
                  U_Poly_.StaticVertices(),
                  loop_->controller().K() * loop_->X_hat()));

      // Limit R back inside the box.
      loop_->mutable_R() =
          CoerceGoal<Scalar>(R_poly_hv, equality_k, equality_w, loop_->R());
    }

    const Eigen::Matrix<Scalar, 2, 1> FF_volts = FF * loop_->R();
    const Eigen::Matrix<Scalar, 2, 1> U_ideal =
        loop_->controller().K() * (loop_->R() - loop_->X_hat()) + FF_volts;

    for (int i = 0; i < 2; i++) {
      loop_->mutable_U()[i] = ::aos::Clip(U_ideal[i], -12, 12);
    }

    if (dt_config_.loop_type == LoopType::OPEN_LOOP) {
      loop_->mutable_X_hat() =
          loop_->plant().A() * loop_->X_hat() + loop_->plant().B() * loop_->U();
    }

    // Housekeeping: set the shifting logging values to zero, because we're not shifting
    left_motor_speed_ = kZero;
    right_motor_speed_ = kZero;
    current_left_velocity_ = kZero;
    current_right_velocity_ = kZero;
  } else {
    const Scalar dt =
        ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(
            dt_config_.dt)
            .count();
    current_left_velocity_ =
        (position_.left_encoder - last_position_.left_encoder) / dt;
    current_right_velocity_ =
        (position_.right_encoder - last_position_.right_encoder) / dt;
    left_motor_speed_ =
        MotorSpeed(dt_config_.left_drive, position_.left_shifter_position,
                   current_left_velocity_, left_gear_);
    right_motor_speed_ =
        MotorSpeed(dt_config_.right_drive, position_.right_shifter_position,
                   current_right_velocity_, right_gear_);

    goal_left_velocity_ = current_left_velocity_;
    goal_right_velocity_ = current_right_velocity_;

    // Any motor is not in gear. Speed match.
    ::Eigen::Matrix<Scalar, 1, 1> R_left;
    ::Eigen::Matrix<Scalar, 1, 1> R_right;
    R_left(0, 0) = left_motor_speed_;
    R_right(0, 0) = right_motor_speed_;

    const Scalar wiggle = (static_cast<Scalar>((counter_ % 30) / 15) - kHalf) *
                          static_cast<Scalar>(8.0);

    loop_->mutable_U(0, 0) = ::aos::Clip(
        (R_left / dt_config_.v)(0, 0) + (IsInGear(left_gear_) ? 0 : wiggle),
        -kTwelve, kTwelve);
    loop_->mutable_U(1, 0) = ::aos::Clip(
        (R_right / dt_config_.v)(0, 0) + (IsInGear(right_gear_) ? 0 : wiggle),
        -kTwelve, kTwelve);
#ifdef __linux__
    loop_->mutable_U() *= kTwelve / voltage_battery;
#else
    (void)voltage_battery;
#endif  // __linux__
  }
}

template <typename Scalar>
void PolyDrivetrain<Scalar>::SetOutput(
    ::frc971::control_loops::drivetrain::OutputT *output) {
  if (output != nullptr) {
    output->left_voltage = loop_->U(0, 0);
    output->right_voltage = loop_->U(1, 0);
    output->left_high = MaybeHigh(left_gear_);
    output->right_high = MaybeHigh(right_gear_);
  }
}

template <typename Scalar>
flatbuffers::Offset<CIMLogging> PolyDrivetrain<Scalar>::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  CIMLogging::Builder builder(*fbb);

  builder.add_left_in_gear(IsInGear(left_gear_));
  builder.add_left_motor_speed(left_motor_speed_);
  builder.add_left_velocity(current_left_velocity_);

  builder.add_right_in_gear(IsInGear(right_gear_));
  builder.add_right_motor_speed(right_motor_speed_);
  builder.add_right_velocity(current_right_velocity_);

  return builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
