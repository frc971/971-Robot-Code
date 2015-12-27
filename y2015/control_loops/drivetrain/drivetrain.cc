#include "y2015/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"

#include "y2015/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2015/control_loops/drivetrain/polydrivetrain_cim_plant.h"
#include "y2015/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/shifter_hall_effect.h"

// A consistent way to mark code that goes away without shifters. It's still
// here because we will have shifters again in the future.
#define HAVE_SHIFTERS 0

using frc971::sensors::gyro_reading;

namespace frc971 {
namespace control_loops {

class DrivetrainMotorsSS {
 public:
  class LimitedDrivetrainLoop : public StateFeedbackLoop<4, 2, 2> {
   public:
    LimitedDrivetrainLoop(StateFeedbackLoop<4, 2, 2> &&loop)
        : StateFeedbackLoop<4, 2, 2>(::std::move(loop)),
        U_Poly_((Eigen::Matrix<double, 4, 2>() << 1, 0,
                 -1, 0,
                 0, 1,
                 0, -1).finished(),
                (Eigen::Matrix<double, 4, 1>() << 12.0, 12.0,
                 12.0, 12.0).finished()) {
      ::aos::controls::HPolytope<0>::Init();
      T << 1, -1, 1, 1;
      T_inverse = T.inverse();
    }

    bool output_was_capped() const {
      return output_was_capped_;
    }

   private:
    virtual void CapU() {
      const Eigen::Matrix<double, 4, 1> error = R() - X_hat();

      if (::std::abs(U(0, 0)) > 12.0 || ::std::abs(U(1, 0)) > 12.0) {
        mutable_U() =
            U() * 12.0 / ::std::max(::std::abs(U(0, 0)), ::std::abs(U(1, 0)));
        LOG_MATRIX(DEBUG, "U is now", U());
        // TODO(Austin): Figure out why the polytope stuff wasn't working and
        // remove this hack.
        output_was_capped_ = true;
        return;

        LOG_MATRIX(DEBUG, "U at start", U());
        LOG_MATRIX(DEBUG, "R at start", R());
        LOG_MATRIX(DEBUG, "Xhat at start", X_hat());

        Eigen::Matrix<double, 2, 2> position_K;
        position_K << K(0, 0), K(0, 2),
                   K(1, 0), K(1, 2);
        Eigen::Matrix<double, 2, 2> velocity_K;
        velocity_K << K(0, 1), K(0, 3),
                   K(1, 1), K(1, 3);

        Eigen::Matrix<double, 2, 1> position_error;
        position_error << error(0, 0), error(2, 0);
        const auto drive_error = T_inverse * position_error;
        Eigen::Matrix<double, 2, 1> velocity_error;
        velocity_error << error(1, 0), error(3, 0);
        LOG_MATRIX(DEBUG, "error", error);

        const auto &poly = U_Poly_;
        const Eigen::Matrix<double, 4, 2> pos_poly_H =
            poly.H() * position_K * T;
        const Eigen::Matrix<double, 4, 1> pos_poly_k =
            poly.k() - poly.H() * velocity_K * velocity_error;
        const ::aos::controls::HPolytope<2> pos_poly(pos_poly_H, pos_poly_k);

        Eigen::Matrix<double, 2, 1> adjusted_pos_error;
        {
          const auto &P = drive_error;

          Eigen::Matrix<double, 1, 2> L45;
          L45 << ::aos::sign(P(1, 0)), -::aos::sign(P(0, 0));
          const double w45 = 0;

          Eigen::Matrix<double, 1, 2> LH;
          if (::std::abs(P(0, 0)) > ::std::abs(P(1, 0))) {
            LH << 0, 1;
          } else {
            LH << 1, 0;
          }
          const double wh = LH.dot(P);

          Eigen::Matrix<double, 2, 2> standard;
          standard << L45, LH;
          Eigen::Matrix<double, 2, 1> W;
          W << w45, wh;
          const Eigen::Matrix<double, 2, 1> intersection =
              standard.inverse() * W;

          bool is_inside_h;
          const auto adjusted_pos_error_h =
              DoCoerceGoal(pos_poly, LH, wh, drive_error, &is_inside_h);
          const auto adjusted_pos_error_45 =
              DoCoerceGoal(pos_poly, L45, w45, intersection, nullptr);
          if (pos_poly.IsInside(intersection)) {
            adjusted_pos_error = adjusted_pos_error_h;
          } else {
            if (is_inside_h) {
              if (adjusted_pos_error_h.norm() > adjusted_pos_error_45.norm()) {
                adjusted_pos_error = adjusted_pos_error_h;
              } else {
                adjusted_pos_error = adjusted_pos_error_45;
              }
            } else {
              adjusted_pos_error = adjusted_pos_error_45;
            }
          }
        }

        LOG_MATRIX(DEBUG, "adjusted_pos_error", adjusted_pos_error);
        mutable_U() =
            velocity_K * velocity_error + position_K * T * adjusted_pos_error;
        LOG_MATRIX(DEBUG, "U is now", U());
      } else {
        output_was_capped_ = false;
      }
    }

    const ::aos::controls::HPolytope<2> U_Poly_;
    Eigen::Matrix<double, 2, 2> T, T_inverse;
    bool output_was_capped_ = false;;
  };

  DrivetrainMotorsSS()
      : loop_(new LimitedDrivetrainLoop(
            constants::GetValues().make_drivetrain_loop())),
        filtered_offset_(0.0),
        gyro_(0.0),
        left_goal_(0.0),
        right_goal_(0.0),
        raw_left_(0.0),
        raw_right_(0.0) {
    // Low gear on both.
    loop_->set_controller_index(0);
  }

  void SetGoal(double left, double left_velocity, double right,
               double right_velocity) {
    left_goal_ = left;
    right_goal_ = right;
    loop_->mutable_R() << left, left_velocity, right, right_velocity;
  }
  void SetRawPosition(double left, double right) {
    raw_right_ = right;
    raw_left_ = left;
    Eigen::Matrix<double, 2, 1> Y;
    Y << left + filtered_offset_, right - filtered_offset_;
    loop_->Correct(Y);
  }
  void SetPosition(double left, double right, double gyro) {
    // Decay the offset quickly because this gyro is great.
    const double offset =
        (right - left - gyro * constants::GetValues().turn_width) / 2.0;
    filtered_offset_ = 0.25 * offset + 0.75 * filtered_offset_;
    gyro_ = gyro;
    SetRawPosition(left, right);
  }

  void SetExternalMotors(double left_voltage, double right_voltage) {
    loop_->mutable_U() << left_voltage, right_voltage;
  }

  void Update(bool stop_motors, bool enable_control_loop) {
    if (enable_control_loop) {
      loop_->Update(stop_motors);
    } else {
      if (stop_motors) {
        loop_->mutable_U().setZero();
        loop_->mutable_U_uncapped().setZero();
      }
      loop_->UpdateObserver(loop_->U());
    }
    ::Eigen::Matrix<double, 4, 1> E = loop_->R() - loop_->X_hat();
    LOG_MATRIX(DEBUG, "E", E);
  }

  double GetEstimatedRobotSpeed() const {
    // lets just call the average of left and right velocities close enough
    return (loop_->X_hat(1, 0) + loop_->X_hat(3, 0)) / 2;
  }

  double GetEstimatedLeftEncoder() const {
    return loop_->X_hat(0, 0);
  }

  double GetEstimatedRightEncoder() const {
    return loop_->X_hat(2, 0);
  }

  bool OutputWasCapped() const {
    return loop_->output_was_capped();
  }

  void SendMotors(DrivetrainQueue::Output *output) const {
    if (output) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
      output->left_high = false;
      output->right_high = false;
    }
  }

  const LimitedDrivetrainLoop &loop() const { return *loop_; }

 private:
  ::std::unique_ptr<LimitedDrivetrainLoop> loop_;

  double filtered_offset_;
  double gyro_;
  double left_goal_;
  double right_goal_;
  double raw_left_;
  double raw_right_;
};

class PolyDrivetrain {
 public:

  enum Gear {
    HIGH,
    LOW,
    SHIFTING_UP,
    SHIFTING_DOWN
  };
  // Stall Torque in N m
  static constexpr double kStallTorque = 2.42;
  // Stall Current in Amps
  static constexpr double kStallCurrent = 133.0;
  // Free Speed in RPM. Used number from last year.
  static constexpr double kFreeSpeed = 4650.0;
  // Free Current in Amps
  static constexpr double kFreeCurrent = 2.7;
  // Moment of inertia of the drivetrain in kg m^2
  // Just borrowed from last year.
  static constexpr double J = 10;
  // Mass of the robot, in kg.
  static constexpr double m = 68;
  // Radius of the robot, in meters (from last year).
  static constexpr double rb = 0.9603 / 2.0;
  static constexpr double kWheelRadius = 0.0515938;
  // Resistance of the motor, divided by the number of motors.
  static constexpr double kR = (12.0 / kStallCurrent / 2 + 0.03) / (0.93 * 0.93);
  // Motor velocity constant
  static constexpr double Kv =
      ((kFreeSpeed / 60.0 * 2.0 * M_PI) / (12.0 - kR * kFreeCurrent));
  // Torque constant
  static constexpr double Kt = kStallTorque / kStallCurrent;

  PolyDrivetrain()
      : U_Poly_((Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
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
        position_time_delta_(0.01),
        left_gear_(LOW),
        right_gear_(LOW),
        counter_(0) {

    last_position_.Zero();
    position_.Zero();
  }
  static bool IsInGear(Gear gear) { return gear == LOW || gear == HIGH; }

  static double MotorSpeed(const constants::ShifterHallEffect &hall_effect,
                           double shifter_position, double velocity) {
    // TODO(austin): G_high, G_low and kWheelRadius
    const double avg_hall_effect =
        (hall_effect.clear_high + hall_effect.clear_low) / 2.0;

    if (shifter_position > avg_hall_effect) {
      return velocity / constants::GetValues().high_gear_ratio / kWheelRadius;
    } else {
      return velocity / constants::GetValues().low_gear_ratio / kWheelRadius;
    }
  }

  Gear ComputeGear(const constants::ShifterHallEffect &hall_effect,
                   double velocity, Gear current) {
    const double low_omega = MotorSpeed(hall_effect, 0.0, ::std::abs(velocity));
    const double high_omega =
        MotorSpeed(hall_effect, 1.0, ::std::abs(velocity));

    double high_torque = ((12.0 - high_omega / Kv) * Kt / kR);
    double low_torque = ((12.0 - low_omega / Kv) * Kt / kR);
    double high_power = high_torque * high_omega;
    double low_power = low_torque * low_omega;

    // TODO(aschuh): Do this right!
    if ((current == HIGH || high_power > low_power + 160) &&
        ::std::abs(velocity) > 0.14) {
      return HIGH;
    } else {
      return LOW;
    }
  }

  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear) {
    const double kWheelNonLinearity = 0.5;
    // Apply a sin function that's scaled to make it feel better.
    const double angular_range = M_PI_2 * kWheelNonLinearity;

    wheel_ = sin(angular_range * wheel) / sin(angular_range);
    wheel_ = sin(angular_range * wheel_) / sin(angular_range);
    wheel_ *= 2.3;
    quickturn_ = quickturn;

    static const double kThrottleDeadband = 0.05;
    if (::std::abs(throttle) < kThrottleDeadband) {
      throttle_ = 0;
    } else {
      throttle_ = copysign((::std::abs(throttle) - kThrottleDeadband) /
                           (1.0 - kThrottleDeadband), throttle);
    }

    // TODO(austin): Fix the upshift logic to include states.
    Gear requested_gear;
    if (false) {
      const auto &values = constants::GetValues();
      const double current_left_velocity =
          (position_.left_encoder - last_position_.left_encoder) /
          position_time_delta_;
      const double current_right_velocity =
          (position_.right_encoder - last_position_.right_encoder) /
          position_time_delta_;

      Gear left_requested =
          ComputeGear(values.left_drive, current_left_velocity, left_gear_);
      Gear right_requested =
          ComputeGear(values.right_drive, current_right_velocity, right_gear_);
      requested_gear =
          (left_requested == HIGH || right_requested == HIGH) ? HIGH : LOW;
    } else {
      requested_gear = highgear ? HIGH : LOW;
    }

    const Gear shift_up =
        constants::GetValues().clutch_transmission ? HIGH : SHIFTING_UP;
    const Gear shift_down =
        constants::GetValues().clutch_transmission ? LOW : SHIFTING_DOWN;

    if (left_gear_ != requested_gear) {
      if (IsInGear(left_gear_)) {
        if (requested_gear == HIGH) {
          left_gear_ = shift_up;
        } else {
          left_gear_ = shift_down;
        }
      } else {
        if (requested_gear == HIGH && left_gear_ == SHIFTING_DOWN) {
          left_gear_ = SHIFTING_UP;
        } else if (requested_gear == LOW && left_gear_ == SHIFTING_UP) {
          left_gear_ = SHIFTING_DOWN;
        }
      }
    }
    if (right_gear_ != requested_gear) {
      if (IsInGear(right_gear_)) {
        if (requested_gear == HIGH) {
          right_gear_ = shift_up;
        } else {
          right_gear_ = shift_down;
        }
      } else {
        if (requested_gear == HIGH && right_gear_ == SHIFTING_DOWN) {
          right_gear_ = SHIFTING_UP;
        } else if (requested_gear == LOW && right_gear_ == SHIFTING_UP) {
          right_gear_ = SHIFTING_DOWN;
        }
      }
    }
  }
  void SetPosition(const DrivetrainQueue::Position *position) {
    const auto &values = constants::GetValues();
    if (position == NULL) {
      ++stale_count_;
    } else {
      last_position_ = position_;
      position_ = *position;
      position_time_delta_ = (stale_count_ + 1) * 0.01;
      stale_count_ = 0;
    }

#if HAVE_SHIFTERS
    if (position) {
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

      // TODO(austin): Constants.
      if (position->left_shifter_position > values.left_drive.clear_high && left_gear_ == SHIFTING_UP) {
        left_gear_ = HIGH;
      }
      if (position->left_shifter_position < values.left_drive.clear_low && left_gear_ == SHIFTING_DOWN) {
        left_gear_ = LOW;
      }
      if (position->right_shifter_position > values.right_drive.clear_high && right_gear_ == SHIFTING_UP) {
        right_gear_ = HIGH;
      }
      if (position->right_shifter_position < values.right_drive.clear_low && right_gear_ == SHIFTING_DOWN) {
        right_gear_ = LOW;
      }

      gear_logging.left_state = left_gear_;
      gear_logging.right_state = right_gear_;
      LOG_STRUCT(DEBUG, "state", gear_logging);
    }
#else
    (void) values;
#endif
  }

  double FilterVelocity(double throttle) {
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

    const double adjusted_ff_voltage = ::aos::Clip(
        throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
    return (adjusted_ff_voltage +
            ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) /
                2.0) /
           (ttrust_ * min_K_sum + min_FF_sum);
  }

  double MaxVelocity() {
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
    //const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
    const double high_min_FF_sum = FF_high.col(0).sum();

    const double adjusted_ff_voltage = ::aos::Clip(
        12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
    return adjusted_ff_voltage / min_FF_sum;
  }

  void Update() {
    const auto &values = constants::GetValues();
    // TODO(austin): Observer for the current velocity instead of difference
    // calculations.
    ++counter_;
#if HAVE_SHIFTERS
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
#else
    (void) values;
#endif

#if HAVE_SHIFTERS
    if (IsInGear(left_gear_) && IsInGear(right_gear_)) {
#else
    {
#endif
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
      LOG(DEBUG, "l=%f r=%f\n", left_velocity, right_velocity);

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

      // TODO(austin): Model this better.
      // TODO(austin): Feed back?
      loop_->mutable_X_hat() =
          loop_->A() * loop_->X_hat() + loop_->B() * loop_->U();
#if HAVE_SHIFTERS
    } else {
      // Any motor is not in gear.  Speed match.
      ::Eigen::Matrix<double, 1, 1> R_left;
      ::Eigen::Matrix<double, 1, 1> R_right;
      R_left(0, 0) = left_motor_speed;
      R_right(0, 0) = right_motor_speed;

      const double wiggle =
          (static_cast<double>((counter_ % 20) / 10) - 0.5) * 5.0;

      loop_->mutable_U(0, 0) = ::aos::Clip(
          (R_left / Kv)(0, 0) + (IsInGear(left_gear_) ? 0 : wiggle),
          -12.0, 12.0);
      loop_->mutable_U(1, 0) = ::aos::Clip(
          (R_right / Kv)(0, 0) + (IsInGear(right_gear_) ? 0 : wiggle),
          -12.0, 12.0);
      loop_->mutable_U() *= 12.0 / ::aos::robot_state->voltage_battery;
#endif
    }
  }

  void SendMotors(DrivetrainQueue::Output *output) {
    if (output != NULL) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
      output->left_high = left_gear_ == HIGH || left_gear_ == SHIFTING_UP;
      output->right_high = right_gear_ == HIGH || right_gear_ == SHIFTING_UP;
    }
  }

 private:
  const ::aos::controls::HPolytope<2> U_Poly_;

  ::std::unique_ptr<StateFeedbackLoop<2, 2, 2>> loop_;

  const double ttrust_;
  double wheel_;
  double throttle_;
  bool quickturn_;
  int stale_count_;
  double position_time_delta_;
  Gear left_gear_;
  Gear right_gear_;
  DrivetrainQueue::Position last_position_;
  DrivetrainQueue::Position position_;
  int counter_;
};
constexpr double PolyDrivetrain::kStallTorque;
constexpr double PolyDrivetrain::kStallCurrent;
constexpr double PolyDrivetrain::kFreeSpeed;
constexpr double PolyDrivetrain::kFreeCurrent;
constexpr double PolyDrivetrain::J;
constexpr double PolyDrivetrain::m;
constexpr double PolyDrivetrain::rb;
constexpr double PolyDrivetrain::kWheelRadius;
constexpr double PolyDrivetrain::kR;
constexpr double PolyDrivetrain::Kv;
constexpr double PolyDrivetrain::Kt;


void DrivetrainLoop::RunIteration(const DrivetrainQueue::Goal *goal,
                                  const DrivetrainQueue::Position *position,
                                  DrivetrainQueue::Output *output,
                                  DrivetrainQueue::Status * status) {
  // TODO(aschuh): These should be members of the class.
  static DrivetrainMotorsSS dt_closedloop;
  static PolyDrivetrain dt_openloop;

  bool bad_pos = false;
  if (position == nullptr) {
    LOG_INTERVAL(no_position_);
    bad_pos = true;
  }
  no_position_.Print();

  bool control_loop_driving = false;
  if (goal) {
    double wheel = goal->steering;
    double throttle = goal->throttle;
    bool quickturn = goal->quickturn;
#if HAVE_SHIFTERS
    bool highgear = goal->highgear;
#endif

    control_loop_driving = goal->control_loop_driving;
    double left_goal = goal->left_goal;
    double right_goal = goal->right_goal;

    dt_closedloop.SetGoal(left_goal, goal->left_velocity_goal, right_goal,
                          goal->right_velocity_goal);
#if HAVE_SHIFTERS
    dt_openloop.SetGoal(wheel, throttle, quickturn, highgear);
#else
    dt_openloop.SetGoal(wheel, throttle, quickturn, false);
#endif
  }

  if (!bad_pos) {
    const double left_encoder = position->left_encoder;
    const double right_encoder = position->right_encoder;
    if (gyro_reading.FetchLatest()) {
      LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
      dt_closedloop.SetPosition(left_encoder, right_encoder,
                                gyro_reading->angle);
    } else {
      dt_closedloop.SetRawPosition(left_encoder, right_encoder);
    }
  }
  dt_openloop.SetPosition(position);
  dt_openloop.Update();

  if (control_loop_driving) {
    dt_closedloop.Update(output == NULL, true);
    dt_closedloop.SendMotors(output);
  } else {
    dt_openloop.SendMotors(output);
    if (output) {
      dt_closedloop.SetExternalMotors(output->left_voltage,
                                      output->right_voltage);
    }
    dt_closedloop.Update(output == NULL, false);
  }

  // set the output status of the control loop state
  if (status) {
    status->robot_speed = dt_closedloop.GetEstimatedRobotSpeed();
    status->filtered_left_position = dt_closedloop.GetEstimatedLeftEncoder();
    status->filtered_right_position = dt_closedloop.GetEstimatedRightEncoder();

    status->filtered_left_velocity = dt_closedloop.loop().X_hat(1, 0);
    status->filtered_right_velocity = dt_closedloop.loop().X_hat(3, 0);
    status->output_was_capped = dt_closedloop.OutputWasCapped();
    status->uncapped_left_voltage = dt_closedloop.loop().U_uncapped(0, 0);
    status->uncapped_right_voltage = dt_closedloop.loop().U_uncapped(1, 0);
  }
}

}  // namespace control_loops
}  // namespace frc971
