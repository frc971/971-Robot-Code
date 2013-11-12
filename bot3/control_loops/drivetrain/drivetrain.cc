#include "bot3/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>

#include "aos/common/logging/logging.h"
#include "aos/common/queue.h"
#include "bot3/control_loops/drivetrain/drivetrain_motor_plant.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"

using frc971::sensors::gyro;
using ::frc971::control_loops::shifters;

namespace bot3 {
namespace control_loops {

// Width of the robot.
const double width = 22.0 / 100.0 * 2.54;

class DrivetrainMotorsOL {
 public:
  DrivetrainMotorsOL() {
    _old_wheel = 0.0;
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
      wheelNonLinearity = 0.1;  // used to be csvReader->TURN_NONLIN_HIGH
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * _wheel) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
    } else {
      wheelNonLinearity = 0.2;  // used to be csvReader->TURN_NONLIN_LOW
      // Apply a sin function that's scaled to make it feel better.
      const double angular_range = M_PI / 2.0 * wheelNonLinearity;
      wheel = sin(angular_range * _wheel) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
      wheel = sin(angular_range * wheel) / sin(angular_range);
    }

    static const double kThrottleDeadband = 0.05;
    if (::std::abs(_throttle) < kThrottleDeadband) {
      _throttle = 0;
    } else {
      _throttle = copysign((::std::abs(_throttle) - kThrottleDeadband) /
                           (1.0 - kThrottleDeadband), _throttle);
    }

    double neg_inertia_scalar;
    if (_highgear) {
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

    linear_power = _throttle;

    if (_quickturn) {
      double qt_angular_power = wheel;
      if (::std::abs(linear_power) < 0.2) {
        if (qt_angular_power > 1) qt_angular_power = 1.0;
        if (qt_angular_power < -1) qt_angular_power = -1.0;
      } else {
        qt_angular_power = 0.0;
      }
      overPower = 1.0;
      if (_highgear) {
        sensitivity = 1.0;
      } else {
        sensitivity = 1.0;
      }
      angular_power = wheel;
    } else {
      overPower = 0.0;
      angular_power = ::std::abs(_throttle) * wheel * sensitivity;
    }
    LOG(DEBUG, "Angular power: %f\n", angular_power);

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
        _left_pwm, _right_pwm, _wheel, _throttle);
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
};

void DrivetrainLoop::RunIteration(const Drivetrain::Goal *goal,
                                  const Drivetrain::Position * /*position*/,
                                  Drivetrain::Output *output,
                                  Drivetrain::Status * /*status*/) {
  // TODO(aschuh): These should be members of the class.
  static DrivetrainMotorsOL dt_openloop;

  double wheel = goal->steering;
  double throttle = goal->throttle;
  bool quickturn = goal->quickturn;
  bool highgear = goal->highgear;

  dt_openloop.SetGoal(wheel, throttle, quickturn, highgear);
  dt_openloop.Update();
  dt_openloop.SendMotors(output);
}

}  // namespace control_loops
}  // namespace bot3
