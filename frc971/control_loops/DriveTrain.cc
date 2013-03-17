#include "frc971/control_loops/DriveTrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>

#include "aos/aos_core.h"
#include "aos/common/logging/logging.h"
#include "aos/common/queue.h"
#include "frc971/control_loops/DriveTrain.mat"
#include "frc971/control_loops/DriveTrain.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"

using frc971::sensors::gyro;

namespace frc971 {
namespace control_loops {

// Width of the robot.
const double width = 22.0 / 100.0 * 2.54;

class DrivetrainMotorsSS : public MatrixClass {
 public:
  DrivetrainMotorsSS (void) {
    MATRIX_INIT;
    _offset = 0;
    _integral_offset = 0;
    _left_goal = 0.0;
    _right_goal = 0.0;
    _raw_left = 0.0;
    _raw_right = 0.0;
  }
  void SetGoal(double left, double left_velocity, double right, double right_velocity) {
    _left_goal = left;
    _right_goal = right;
    R << left + _integral_offset * width / 2.0, left_velocity, right - _integral_offset * width / 2.0, right_velocity;
  }
  void SetRawPosition(double left, double right) {
    _raw_right = right;
    _raw_left = left;
    Y << left + _offset + _integral_offset, right - _offset + _integral_offset;
  }
  void SetPosition(double left, double right, double gyro, bool control_loop_driving) {
    // Decay the offset quickly because this gyro is great.
    _offset = (0.25) * (right - left - gyro * width) / 2.0 + 0.75 * _offset;
    const double angle_error = (_right_goal - _left_goal) / width - (_raw_right - _offset - _raw_left - _offset) / width;
    if (!control_loop_driving) {
      _integral_offset = 0.0;
    } else if (std::abs(angle_error) < M_PI / 10.0) {
      _integral_offset -= angle_error * 0.010;
    } else {
      _integral_offset *= 0.97;
    }
    _gyro = gyro;
    SetRawPosition(left, right);
    LOG(DEBUG, "Left %f->%f Right %f->%f Gyro %f aerror %f ioff %f\n", left + _offset, _left_goal, right - _offset, _right_goal, gyro, angle_error, _integral_offset);
  }
  double UnDeadband(double value) {
    const double positive_deadband_power = 0.15 * 12;
    const double negative_deadband_power = 0.09 * 12;
    if (value > 0) {
      value += positive_deadband_power;
    }
    if (value < 0) {
      value -= negative_deadband_power;
    }
    if (value > 12.0) {
      value = 12.0;
    }
    if (value < -12.0) {
      value = -12.0;
    }
    return value;
  }

  void SendMotors(Drivetrain::Output *status) {
    if (status) {
      status->left_voltage = UnDeadband(U[0]);
      status->right_voltage = UnDeadband(U[1]);
    }
  }
  void PrintMotors() const {
    // LOG(DEBUG, "Left Power %f Right Power %f lg %f rg %f le %f re %f gyro %f\n", U[0], U[1], R[0], R[2], Y[0], Y[1], _gyro);
    LOG(DEBUG, "lg %f rg %f le %f re %f gyro %f off %f\n", R[0], R[2], Y[0], Y[1], _gyro * 180.0 / M_PI, _offset);
  }

 private:
  double _integral_offset;
  double _offset;
  double _gyro;
  double _left_goal;
  double _right_goal;
  double _raw_left;
  double _raw_right;
};

class DrivetrainMotorsOL {
 public:
  DrivetrainMotorsOL() {
    _old_wheel = 0.0;
    quick_stop_accumulator = 0.0;
    _wheel = 0.0;
    _throttle = 0.0;
    _quickturn = false;
    _highgear = true;
    _neg_inertia_accumulator = 0.0;
    _left_pwm = 0.0;
    _right_pwm = 0.0;
  }
  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear) {
    _wheel = wheel;
    _throttle = throttle;
    _quickturn = quickturn;
    _highgear = highgear;
    _left_pwm = 0.0;
    _right_pwm = 0.0;
  }
  void Update(void) {
    double overPower;
    float sensitivity = 1.7;
    float angular_power;
    float linear_power;
    double wheel;

    double neg_inertia = _wheel - _old_wheel;
    _old_wheel = _wheel;

    double wheelNonLinearity;
    if (_highgear) {
      wheelNonLinearity = 0.7;  // used to be csvReader->TURN_NONLIN_HIGH
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * _wheel) / sin(angular_range);
      wheel = sin(angular_range * _wheel) / sin(angular_range);
    } else {
      wheelNonLinearity = 0.4;  // used to be csvReader->TURN_NONLIN_LOW
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * _wheel) / sin(angular_range);
      wheel = sin(angular_range * _wheel) / sin(angular_range);
      wheel = sin(angular_range * _wheel) / sin(angular_range);
    }

    double neg_inertia_scalar;
    if (_highgear) {
      neg_inertia_scalar = 20.0;  // used to be csvReader->NEG_INTERTIA_HIGH
      sensitivity = 1.22; // used to be csvReader->SENSE_HIGH
    } else {
      if (wheel * neg_inertia > 0) {
        neg_inertia_scalar = 16;  // used to be csvReader->NEG_INERTIA_LOW_MORE
      } else {
        if (fabs(wheel) > 0.65) {
          neg_inertia_scalar = 16;  // used to be csvReader->NEG_INTERTIA_LOW_LESS_EXT
        } else {
          neg_inertia_scalar = 5;  // used to be csvReader->NEG_INTERTIA_LOW_LESS
        }
      }
      sensitivity = 1.24;  // used to be csvReader->SENSE_LOW

      if (fabs(_throttle) > 0.1) {  // used to be csvReader->SENSE_CUTTOFF
        sensitivity = 1 - (1 - sensitivity) / fabs(_throttle);
      }
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

    linear_power = _throttle;

    const double quickstop_scalar = 6;
    if (_quickturn) {
      double qt_angular_power = wheel;
      const double alpha = 0.1;
      if (fabs(linear_power) < 0.2) {
        if (qt_angular_power > 1) qt_angular_power = 1.0;
        if (qt_angular_power < -1) qt_angular_power = -1.0;
      } else {
        qt_angular_power = 0.0;
      }
      quick_stop_accumulator = (1 - alpha) * quick_stop_accumulator + alpha * qt_angular_power * quickstop_scalar;
      overPower = 1.0;
      if (_highgear) {
        sensitivity = 1.0;
      } else {
        sensitivity = 1.0;
      }
      angular_power = wheel;
    } else {
      overPower = 0.0;
      angular_power = fabs(_throttle) * wheel * sensitivity;
      angular_power -= quick_stop_accumulator;
      if (quick_stop_accumulator > 1) {
        quick_stop_accumulator -= 1;
      } else if (quick_stop_accumulator < -1) {
        quick_stop_accumulator += 1;
      } else {
        quick_stop_accumulator = 0;
      }
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
    LOG(DEBUG, "left pwm: %f right pwm: %f wheel: %f throttle: %f, qa %f\n",
        _left_pwm, _right_pwm, _wheel, _throttle, quick_stop_accumulator);
    if (output) {
      output->left_voltage = _left_pwm * 12.0;
      output->right_voltage = _right_pwm * 12.0;
    }
    if (_highgear) {
      shifters.MakeWithBuilder().set(false).Send();
    } else {
      shifters.MakeWithBuilder().set(true).Send();
    }
  }

 private:
  double _old_wheel;
  double _wheel;
  double _throttle;
  bool _quickturn;
  bool _highgear;
  double _neg_inertia_accumulator;
  double _left_pwm;
  double _right_pwm;
  double quick_stop_accumulator;
};

void DrivetrainLoop::RunIteration(const Drivetrain::Goal *goal,
                                  const Drivetrain::Position *position,
                                  Drivetrain::Output *output,
                                  Drivetrain::Status * /*status*/) {
  // TODO(aschuh): These should be members of the class.
  static DrivetrainMotorsSS dt_closedloop;
  static DrivetrainMotorsOL dt_openloop;

  bool bad_pos = false;
  if (position == NULL) {
    LOG(WARNING, "no pos\n");
    bad_pos = true;
  }

  double wheel = goal->steering;
  double throttle = goal->throttle;
  bool quickturn = goal->quickturn;
  bool highgear = goal->highgear;

  bool control_loop_driving = goal->control_loop_driving;
  double left_goal = goal->left_goal;
  double right_goal = goal->right_goal;

  dt_closedloop.SetGoal(left_goal, 0.0, right_goal, 0.0);
  if (!bad_pos) {
    const double left_encoder = position->left_encoder;
    const double right_encoder = position->right_encoder;
    if (gyro.FetchLatest()) {
      dt_closedloop.SetPosition(left_encoder, right_encoder,
          gyro->angle, control_loop_driving);
    } else {
      dt_closedloop.SetRawPosition(left_encoder, right_encoder);
    }
  }
  dt_closedloop.Update(!bad_pos, bad_pos || (output == NULL));
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

AOS_RUN_LOOP(frc971::control_loops::DrivetrainLoop)
