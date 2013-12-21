#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/queue.h"
#include "aos/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/polydrivetrain_cim_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"
#include "frc971/constants.h"

using frc971::sensors::gyro;

namespace frc971 {
namespace control_loops {

// Width of the robot.
const double width = 22.0 / 100.0 * 2.54;

Eigen::Matrix<double, 2, 1> CoerceGoal(aos::controls::HPolytope<2> &region,
                                       const Eigen::Matrix<double, 1, 2> &K,
                                       double w,
                                       const Eigen::Matrix<double, 2, 1> &R) {
  if (region.IsInside(R)) {
    return R;
  }
  Eigen::Matrix<double, 2, 1> parallel_vector;
  Eigen::Matrix<double, 2, 1> perpendicular_vector;
  perpendicular_vector = K.transpose().normalized();
  parallel_vector << perpendicular_vector(1, 0), -perpendicular_vector(0, 0);

  aos::controls::HPolytope<1> t_poly(
      region.H() * parallel_vector,
      region.k() - region.H() * perpendicular_vector * w);

  Eigen::Matrix<double, 1, Eigen::Dynamic> vertices = t_poly.Vertices();
  if (vertices.innerSize() > 0) {
    double min_distance_sqr = 0;
    Eigen::Matrix<double, 2, 1> closest_point;
    for (int i = 0; i < vertices.innerSize(); i++) {
      Eigen::Matrix<double, 2, 1> point;
      point = parallel_vector * vertices(0, i) + perpendicular_vector * w;
      const double length = (R - point).squaredNorm();
      if (i == 0 || length < min_distance_sqr) {
        closest_point = point;
        min_distance_sqr = length;
      }
    }
    return closest_point;
  } else {
    Eigen::Matrix<double, 2, Eigen::Dynamic> region_vertices =
        region.Vertices();
    double min_distance;
    int closest_i = 0;
    for (int i = 0; i < region_vertices.outerSize(); i++) {
      const double length = ::std::abs(
          (perpendicular_vector.transpose() * (region_vertices.col(i)))(0, 0));
      if (i == 0 || length < min_distance) {
        closest_i = i;
        min_distance = length;
      }
    }
    return region_vertices.col(closest_i);
  }
}

class DrivetrainMotorsSS {
 public:
  DrivetrainMotorsSS()
      : loop_(new StateFeedbackLoop<4, 2, 2>(
            constants::GetValues().make_drivetrain_loop())) {
    _offset = 0;
    _integral_offset = 0;
    _left_goal = 0.0;
    _right_goal = 0.0;
    _raw_left = 0.0;
    _raw_right = 0.0;
    _control_loop_driving = false;
  }
  void SetGoal(double left, double left_velocity, double right, double right_velocity) {
    _left_goal = left;
    _right_goal = right;
    loop_->R << left, left_velocity, right, right_velocity;
  }
  void SetRawPosition(double left, double right) {
    _raw_right = right;
    _raw_left = left;
    loop_->Y << left, right;
  }
  void SetPosition(
      double left, double right, double gyro, bool control_loop_driving) {
    // Decay the offset quickly because this gyro is great.
    _offset = (0.25) * (right - left - gyro * width) / 2.0 + 0.75 * _offset;
    //const double angle_error = (_right_goal - _left_goal) / width - (_raw_right - _offset - _raw_left - _offset) / width;
    // TODO(aschuh): Add in the gyro.
    _integral_offset = 0.0;
    _offset = 0.0;
    _gyro = gyro;
    _control_loop_driving = control_loop_driving;
    SetRawPosition(left, right);
  }

  void Update(bool update_observer, bool stop_motors) {
    loop_->Update(update_observer, stop_motors);
  }

  void SendMotors(Drivetrain::Output *output) {
    if (output) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
    }
  }
  void PrintMotors() const {
    // LOG(DEBUG, "Left Power %f Right Power %f lg %f rg %f le %f re %f gyro %f\n", U[0], U[1], R[0], R[2], Y[0], Y[1], _gyro);
    ::Eigen::Matrix<double, 4, 1> E = loop_->R - loop_->X_hat;
    LOG(DEBUG, "E[0, 0]: %f E[1, 0] %f E[2, 0] %f E[3, 0] %f\n", E(0, 0), E(1, 0), E(2, 0), E(3, 0));
  }

 private:
  ::std::unique_ptr<StateFeedbackLoop<4, 2, 2>> loop_;

  double _integral_offset;
  double _offset;
  double _gyro;
  double _left_goal;
  double _right_goal;
  double _raw_left;
  double _raw_right;
  bool _control_loop_driving;
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
  static constexpr double kStallCurrent = 133;
  // Free Speed in RPM. Used number from last year.
  static constexpr double kFreeSpeed = 4650.0;
  // Free Current in Amps
  static constexpr double kFreeCurrent = 2.7;
  // Moment of inertia of the drivetrain in kg m^2
  // Just borrowed from last year.
  static constexpr double J = 6.4;
  // Mass of the robot, in kg.
  static constexpr double m = 68;
  // Radius of the robot, in meters (from last year).
  static constexpr double rb = 0.617998644 / 2.0;
  static constexpr double kWheelRadius = 0.04445;
  // Resistance of the motor, divided by the number of motors.
  static constexpr double kR = (12.0 / kStallCurrent / 4 + 0.03) / (0.93 * 0.93);
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
        left_cim_(new StateFeedbackLoop<1, 1, 1>(MakeCIMLoop())),
        right_cim_(new StateFeedbackLoop<1, 1, 1>(MakeCIMLoop())),
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

  static double MotorSpeed(double shifter_position, double velocity) {
    // TODO(austin): G_high, G_low and kWheelRadius
    if (shifter_position > 0.57) {
      return velocity / constants::GetValues().high_gear_ratio / kWheelRadius;
    } else {
      return velocity / constants::GetValues().low_gear_ratio / kWheelRadius;
    }
  }

  Gear ComputeGear(double velocity, Gear current) {
    const double low_omega = MotorSpeed(0, ::std::abs(velocity));
    const double high_omega = MotorSpeed(1.0, ::std::abs(velocity));

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
      throttle_ = copysign((::std::abs(throttle) - kThrottleDeadband) /
                           (1.0 - kThrottleDeadband), throttle);
    }

    // TODO(austin): Fix the upshift logic to include states.
    Gear requested_gear;
    if (false) {
      const double current_left_velocity =
          (position_.left_encoder - last_position_.left_encoder) /
          position_time_delta_;
      const double current_right_velocity =
          (position_.right_encoder - last_position_.right_encoder) /
          position_time_delta_;

      Gear left_requested = ComputeGear(current_left_velocity, left_gear_);
      Gear right_requested = ComputeGear(current_right_velocity, right_gear_);
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
  void SetPosition(const Drivetrain::Position *position) {
    if (position == NULL) {
      ++stale_count_;
    } else {
      last_position_ = position_;
      position_ = *position;
      position_time_delta_ = (stale_count_ + 1) * 0.01;
      stale_count_ = 0;
    }

    if (position) {
      // Switch to the correct controller.
      // TODO(austin): Un-hard code 0.57
      if (position->left_shifter_position < 0.57) {
        if (position->right_shifter_position < 0.57 || right_gear_ == LOW) {
          LOG(DEBUG, "Loop Left low, Right low\n");
          loop_->set_controller_index(0);
        } else {
          LOG(DEBUG, "Loop Left low, Right high\n");
          loop_->set_controller_index(1);
        }
      } else {
        if (position->right_shifter_position < 0.57 || left_gear_ == LOW) {
          LOG(DEBUG, "Loop Left high, Right low\n");
          loop_->set_controller_index(2);
        } else {
          LOG(DEBUG, "Loop Left high, Right high\n");
          loop_->set_controller_index(3);
        }
      }
      switch (left_gear_) {
        case LOW:
          LOG(DEBUG, "Left is in low\n");
          break;
        case HIGH:
          LOG(DEBUG, "Left is in high\n");
          break;
        case SHIFTING_UP:
          LOG(DEBUG, "Left is shifting up\n");
          break;
        case SHIFTING_DOWN:
          LOG(DEBUG, "Left is shifting down\n");
          break;
      }
      switch (right_gear_) {
        case LOW:
          LOG(DEBUG, "Right is in low\n");
          break;
        case HIGH:
          LOG(DEBUG, "Right is in high\n");
          break;
        case SHIFTING_UP:
          LOG(DEBUG, "Right is shifting up\n");
          break;
        case SHIFTING_DOWN:
          LOG(DEBUG, "Right is shifting down\n");
          break;
      }
      // TODO(austin): Constants.
      if (position->left_shifter_position > 0.9 && left_gear_ == SHIFTING_UP) {
        left_gear_ = HIGH;
      }
      if (position->left_shifter_position < 0.1 && left_gear_ == SHIFTING_DOWN) {
        left_gear_ = LOW;
      }
      if (position->right_shifter_position > 0.9 && right_gear_ == SHIFTING_UP) {
        right_gear_ = HIGH;
      }
      if (position->right_shifter_position < 0.1 && right_gear_ == SHIFTING_DOWN) {
        right_gear_ = LOW;
      }
    }
  }

  double FilterVelocity(double throttle) {
    const Eigen::Matrix<double, 2, 2> FF =
        loop_->B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

    constexpr int kHighGearController = 3;
    const Eigen::Matrix<double, 2, 2> FF_high =
        loop_->controller(kHighGearController).plant.B.inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() -
         loop_->controller(kHighGearController).plant.A);

    ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
    int min_FF_sum_index;
    const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
    const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
    const double high_min_FF_sum = FF_high.col(0).sum();

    const double adjusted_ff_voltage = ::aos::Clip(
        throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
    return ((adjusted_ff_voltage +
             ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) /
                 2.0) /
            (ttrust_ * min_K_sum + min_FF_sum));
  }

  double MaxVelocity() {
    const Eigen::Matrix<double, 2, 2> FF =
        loop_->B().inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

    constexpr int kHighGearController = 3;
    const Eigen::Matrix<double, 2, 2> FF_high =
        loop_->controller(kHighGearController).plant.B.inverse() *
        (Eigen::Matrix<double, 2, 2>::Identity() -
         loop_->controller(kHighGearController).plant.A);

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
        MotorSpeed(position_.left_shifter_position, current_left_velocity);
    const double right_motor_speed =
        MotorSpeed(position_.right_shifter_position, current_right_velocity);

    // Reset the CIM model to the current conditions to be ready for when we shift.
    if (IsInGear(left_gear_)) {
      left_cim_->X_hat(0, 0) = left_motor_speed;
      LOG(DEBUG, "Setting left CIM to %f at robot speed %f\n", left_motor_speed,
          current_left_velocity);
    }
    if (IsInGear(right_gear_)) {
      right_cim_->X_hat(0, 0) = right_motor_speed;
      LOG(DEBUG, "Setting right CIM to %f at robot speed %f\n",
          right_motor_speed, current_right_velocity);
    }
    LOG(DEBUG, "robot speed l=%f r=%f\n", current_left_velocity,
        current_right_velocity);

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
      loop_->R << left_velocity, right_velocity;

      if (!quickturn_) {
        // K * R = w
        Eigen::Matrix<double, 1, 2> equality_k;
        equality_k << 1 + sign_svel, -(1 - sign_svel);
        const double equality_w = 0.0;

        // Construct a constraint on R by manipulating the constraint on U
        ::aos::controls::HPolytope<2> R_poly = ::aos::controls::HPolytope<2>(
            U_Poly_.H() * (loop_->K() + FF),
            U_Poly_.k() + U_Poly_.H() * loop_->K() * loop_->X_hat);

        // Limit R back inside the box.
        loop_->R = CoerceGoal(R_poly, equality_k, equality_w, loop_->R);
      }

      const Eigen::Matrix<double, 2, 1> FF_volts = FF * loop_->R;
      const Eigen::Matrix<double, 2, 1> U_ideal =
          loop_->K() * (loop_->R - loop_->X_hat) + FF_volts;

      for (int i = 0; i < 2; i++) {
        loop_->U[i] = ::aos::Clip(U_ideal[i], -12, 12);
      }

      // TODO(austin): Model this better.
      // TODO(austin): Feed back?
      loop_->X_hat = loop_->A() * loop_->X_hat + loop_->B() * loop_->U;
    } else {
      // Any motor is not in gear.  Speed match.
      ::Eigen::Matrix<double, 1, 1> R_left;
      R_left(0, 0) = left_motor_speed;
      const double wiggle = (static_cast<double>((counter_ % 4) / 2) - 0.5) * 3.5;

      loop_->U(0, 0) =
          ::aos::Clip((R_left / Kv)(0, 0) + wiggle, -position_.battery_voltage,
                      position_.battery_voltage);
      right_cim_->X_hat = right_cim_->A() * right_cim_->X_hat +
                          right_cim_->B() * loop_->U(0, 0);

      ::Eigen::Matrix<double, 1, 1> R_right;
      R_right(0, 0) = right_motor_speed;
      loop_->U(1, 0) =
          ::aos::Clip((R_right / Kv)(0, 0) + wiggle, -position_.battery_voltage,
                      position_.battery_voltage);
      right_cim_->X_hat = right_cim_->A() * right_cim_->X_hat +
                          right_cim_->B() * loop_->U(1, 0);
      loop_->U *= 12.0 / position_.battery_voltage;
    }
  }

  void SendMotors(Drivetrain::Output *output) {
    if (output != NULL) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
    }
    // Go in high gear if anything wants to be in high gear.
    // TODO(austin): Seperate these.
    if (left_gear_ == HIGH || left_gear_ == SHIFTING_UP ||
        right_gear_ == HIGH || right_gear_ == SHIFTING_UP) {
      shifters.MakeWithBuilder().set(false).Send();
    } else {
      shifters.MakeWithBuilder().set(true).Send();
    }
  }

 private:
  const ::aos::controls::HPolytope<2> U_Poly_;

  ::std::unique_ptr<StateFeedbackLoop<2, 2, 2>> loop_;
  ::std::unique_ptr<StateFeedbackLoop<1, 1, 1>> left_cim_;
  ::std::unique_ptr<StateFeedbackLoop<1, 1, 1>> right_cim_;

  const double ttrust_;
  double wheel_;
  double throttle_;
  bool quickturn_;
  int stale_count_;
  double position_time_delta_;
  Gear left_gear_;
  Gear right_gear_;
  Drivetrain::Position last_position_;
  Drivetrain::Position position_;
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



class DrivetrainMotorsOL {
 public:
  DrivetrainMotorsOL() {
    _old_wheel = 0.0;
    wheel_ = 0.0;
    throttle_ = 0.0;
    quickturn_ = false;
    highgear_ = true;
    _neg_inertia_accumulator = 0.0;
    _left_pwm = 0.0;
    _right_pwm = 0.0;
  }
  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear) {
    wheel_ = wheel;
    throttle_ = throttle;
    quickturn_ = quickturn;
    highgear_ = highgear;
    _left_pwm = 0.0;
    _right_pwm = 0.0;
  }
  void Update() {
    double overPower;
    float sensitivity = 1.7;
    float angular_power;
    float linear_power;
    double wheel;

    double neg_inertia = wheel_ - _old_wheel;
    _old_wheel = wheel_;

    double wheelNonLinearity;
    if (highgear_) {
      wheelNonLinearity = 0.1;  // used to be csvReader->TURN_NONLIN_HIGH
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * wheel_) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
    } else {
      wheelNonLinearity = 0.2;  // used to be csvReader->TURN_NONLIN_LOW
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * wheel_) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
    }

    static const double kThrottleDeadband = 0.05;
    if (::std::abs(throttle_) < kThrottleDeadband) {
      throttle_ = 0;
    } else {
      throttle_ = copysign((::std::abs(throttle_) - kThrottleDeadband) /
                           (1.0 - kThrottleDeadband), throttle_);
    }

    double neg_inertia_scalar;
    if (highgear_) {
      neg_inertia_scalar = 8.0;  // used to be csvReader->NEG_INTERTIA_HIGH
      sensitivity = 1.22; // used to be csvReader->SENSE_HIGH
    } else {
      if (wheel * neg_inertia > 0) {
        neg_inertia_scalar = 5;  // used to be csvReader->NEG_INERTIA_LOW_MORE
      } else {
        if (::std::abs(wheel) > 0.65) {
          neg_inertia_scalar = 5;  // used to be csvReader->NEG_INTERTIA_LOW_LESS_EXT
        } else {
          neg_inertia_scalar = 5;  // used to be csvReader->NEG_INTERTIA_LOW_LESS
        }
      }
      sensitivity = 1.24;  // used to be csvReader->SENSE_LOW
    }
    double neg_inertia_power = neg_inertia * neg_inertia_scalar;
    _neg_inertia_accumulator += neg_inertia_power;

    wheel = wheel + _neg_inertia_accumulator;
    if (_neg_inertia_accumulator > 1) {
      _neg_inertia_accumulator -= 1;
    } else if (_neg_inertia_accumulator < -1) {
      _neg_inertia_accumulator += 1;
    } else {
      _neg_inertia_accumulator = 0;
    }

    linear_power = throttle_;

    if (quickturn_) {
      double qt_angular_power = wheel;
      if (::std::abs(linear_power) < 0.2) {
        if (qt_angular_power > 1) qt_angular_power = 1.0;
        if (qt_angular_power < -1) qt_angular_power = -1.0;
      } else {
        qt_angular_power = 0.0;
      }
      overPower = 1.0;
      if (highgear_) {
        sensitivity = 1.0;
      } else {
        sensitivity = 1.0;
      }
      angular_power = wheel;
    } else {
      overPower = 0.0;
      angular_power = ::std::abs(throttle_) * wheel * sensitivity;
    }

    _right_pwm = _left_pwm = linear_power;
    _left_pwm += angular_power;
    _right_pwm -= angular_power;

    if (_left_pwm > 1.0) {
      _right_pwm -= overPower*(_left_pwm - 1.0);
      _left_pwm = 1.0;
    } else if (_right_pwm > 1.0) {
      _left_pwm -= overPower*(_right_pwm - 1.0);
      _right_pwm = 1.0;
    } else if (_left_pwm < -1.0) {
      _right_pwm += overPower*(-1.0 - _left_pwm);
      _left_pwm = -1.0;
    } else if (_right_pwm < -1.0) {
      _left_pwm += overPower*(-1.0 - _right_pwm);
      _right_pwm = -1.0;
    }
  }

  void SendMotors(Drivetrain::Output *output) {
    LOG(DEBUG, "left pwm: %f right pwm: %f wheel: %f throttle: %f\n",
        _left_pwm, _right_pwm, wheel_, throttle_);
    if (output) {
      output->left_voltage = _left_pwm * 12.0;
      output->right_voltage = _right_pwm * 12.0;
    }
    if (highgear_) {
      shifters.MakeWithBuilder().set(false).Send();
    } else {
      shifters.MakeWithBuilder().set(true).Send();
    }
  }

 private:
  double _old_wheel;
  double wheel_;
  double throttle_;
  bool quickturn_;
  bool highgear_;
  double _neg_inertia_accumulator;
  double _left_pwm;
  double _right_pwm;
};

void DrivetrainLoop::RunIteration(const Drivetrain::Goal *goal,
                                  const Drivetrain::Position *position,
                                  Drivetrain::Output *output,
                                  Drivetrain::Status * /*status*/) {
  // TODO(aschuh): These should be members of the class.
  static DrivetrainMotorsSS dt_closedloop;
  static PolyDrivetrain dt_openloop;

  bool bad_pos = false;
  if (position == nullptr) {
    LOG(WARNING, "no position\n");
    bad_pos = true;
  }

  double wheel = goal->steering;
  double throttle = goal->throttle;
  bool quickturn = goal->quickturn;
  bool highgear = goal->highgear;

  bool control_loop_driving = goal->control_loop_driving;
  double left_goal = goal->left_goal;
  double right_goal = goal->right_goal;

  dt_closedloop.SetGoal(left_goal, goal->left_velocity_goal, right_goal,
                        goal->right_velocity_goal);
  if (!bad_pos) {
    const double left_encoder = position->left_encoder;
    const double right_encoder = position->right_encoder;
    if (gyro.FetchLatest()) {
      LOG(DEBUG, "gyro %f\n", gyro->angle);
      dt_closedloop.SetPosition(left_encoder, right_encoder, gyro->angle,
                                control_loop_driving);
    } else {
      dt_closedloop.SetRawPosition(left_encoder, right_encoder);
    }
  }
  dt_openloop.SetPosition(position);
  dt_closedloop.Update(position, output == NULL);
  dt_openloop.SetGoal(wheel, throttle, quickturn, highgear);
  dt_openloop.Update();
  if (control_loop_driving) {
    dt_closedloop.SendMotors(output);
  } else {
    dt_openloop.SendMotors(output);
  }
}

}  // namespace control_loops
}  // namespace frc971
