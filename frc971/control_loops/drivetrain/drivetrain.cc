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
#include "aos/common/logging/queue_logging.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/polydrivetrain_cim_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/other_sensors.q.h"
#include "frc971/constants.h"

using frc971::sensors::gyro_reading;

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
    double min_distance = INFINITY;
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
    Eigen::Matrix<double, 2, 1> Y;
    Y << left, right;
    loop_->Correct(Y);
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

  void SetExternalMotors(double left_voltage, double right_voltage) {
    loop_->U << left_voltage, right_voltage;
  }

  void Update(bool stop_motors) {
    if (_control_loop_driving) {
      loop_->Update(stop_motors);
    } else {
      if (stop_motors) {
        loop_->U.setZero();
        loop_->U_uncapped.setZero();
      }
      loop_->UpdateObserver();
    }
  }

  double GetEstimatedRobotSpeed() {
    // lets just call the average of left and right velocities close enough
    return (loop_->X_hat(1, 0) + loop_->X_hat(3, 0)) / 2;
  }
  
  double GetEstimatedLeftEncoder() {
    // lets just call the average of left and right velocities close enough
    return loop_->X_hat(0, 0);
  }
  
  double GetEstimatedRightEncoder() {
    return loop_->X_hat(2, 0);
  }

  void SendMotors(Drivetrain::Output *output) {
    if (output) {
      output->left_voltage = loop_->U(0, 0);
      output->right_voltage = loop_->U(1, 0);
    }
  }
  void PrintMotors() const {
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
  void SetPosition(const Drivetrain::Position *position) {
    const auto &values = constants::GetValues();
    if (position == NULL) {
      ++stale_count_;
    } else {
      last_position_ = position_;
      position_ = *position;
      position_time_delta_ = (stale_count_ + 1) * 0.01;
      stale_count_ = 0;
    }

    if (position) {
      GearLogging gear_logging;
      // Switch to the correct controller.
      const double left_middle_shifter_position =
          (values.left_drive.clear_high + values.left_drive.clear_low) / 2.0;
      const double right_middle_shifter_position =
          (values.right_drive.clear_high + values.right_drive.clear_low) / 2.0;

      if (position->left_shifter_position < left_middle_shifter_position) {
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
            left_gear_ == LOW) {
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
      ::Eigen::Matrix<double, 1, 1> R_right;
      R_left(0, 0) = left_motor_speed;
      R_right(0, 0) = right_motor_speed;

      const double wiggle =
          (static_cast<double>((counter_ % 10) / 5) - 0.5) * 5.0;

      loop_->U(0, 0) = ::aos::Clip((R_left / Kv)(0, 0) + wiggle, -12.0, 12.0);
      loop_->U(1, 0) = ::aos::Clip((R_right / Kv)(0, 0) + wiggle, -12.0, 12.0);
    }
  }

  void SendMotors(Drivetrain::Output *output) {
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


void DrivetrainLoop::RunIteration(const Drivetrain::Goal *goal,
                                  const Drivetrain::Position *position,
                                  Drivetrain::Output *output,
                                  Drivetrain::Status * status) {
  // TODO(aschuh): These should be members of the class.
  static DrivetrainMotorsSS dt_closedloop;
  static PolyDrivetrain dt_openloop;

  bool bad_pos = false;
  if (position == nullptr) {
    LOG_INTERVAL(no_position_);
    bad_pos = true;
  }
  no_position_.Print();

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
    if (gyro_reading.FetchLatest()) {
      LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
      dt_closedloop.SetPosition(left_encoder, right_encoder,
                                gyro_reading->angle, control_loop_driving);
    } else {
      dt_closedloop.SetRawPosition(left_encoder, right_encoder);
    }
  }
  dt_openloop.SetPosition(position);
  dt_openloop.SetGoal(wheel, throttle, quickturn, highgear);
  dt_openloop.Update();

  if (control_loop_driving) {
    dt_closedloop.Update(output == NULL);
    dt_closedloop.SendMotors(output);
  } else {
    dt_openloop.SendMotors(output);
    if (output) {
      dt_closedloop.SetExternalMotors(output->left_voltage,
                                      output->right_voltage);
    }
    dt_closedloop.Update(output == NULL);
  }
  
  // set the output status of the controll loop state
  if (status) {
    bool done = false;
    if (goal) {
      done = ((::std::abs(goal->left_goal -
                          dt_closedloop.GetEstimatedLeftEncoder()) <
               constants::GetValues().drivetrain_done_distance) &&
              (::std::abs(goal->right_goal -
                          dt_closedloop.GetEstimatedRightEncoder()) <
               constants::GetValues().drivetrain_done_distance));
    }
    status->is_done = done;
    status->robot_speed = dt_closedloop.GetEstimatedRobotSpeed();
  }
}

}  // namespace control_loops
}  // namespace frc971
