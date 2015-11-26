#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_

#include "aos/common/controls/polytope.h"

#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

namespace frc971 {
namespace control_loops {

class PolyDrivetrain {
 public:
  enum Gear { HIGH, LOW, SHIFTING_UP, SHIFTING_DOWN };
  // Stall Torque in N m
  static constexpr double kStallTorque =
      ::y2014::control_loops::drivetrain::kStallTorque;
  // Stall Current in Amps
  static constexpr double kStallCurrent = ::y2014::control_loops::drivetrain::kStallCurrent;
  // Free Speed in RPM. Used number from last year.
  static constexpr double kFreeSpeed =
      ::y2014::control_loops::drivetrain::kFreeSpeedRPM;
  // Free Current in Amps
  static constexpr double kFreeCurrent =
      ::y2014::control_loops::drivetrain::kFreeCurrent;
  static constexpr double kWheelRadius =
      ::y2014::control_loops::drivetrain::kWheelRadius;
  // Resistance of the motor, divided by the number of motors.
  static constexpr double kR = ::y2014::control_loops::drivetrain::kR;
  // Motor velocity constant
  static constexpr double Kv = ::y2014::control_loops::drivetrain::kV;

  // Torque constant
  static constexpr double Kt = ::y2014::control_loops::drivetrain::kT;

  PolyDrivetrain();

  static bool IsInGear(Gear gear) { return gear == LOW || gear == HIGH; }

  static double MotorSpeed(const constants::ShifterHallEffect &hall_effect,
                           double shifter_position, double velocity);

  Gear ComputeGear(const constants::ShifterHallEffect &hall_effect,
                   double velocity, Gear current);

  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear);

  void SetPosition(const DrivetrainQueue::Position *position);

  double FilterVelocity(double throttle);

  double MaxVelocity();

  void Update();

  void SendMotors(DrivetrainQueue::Output *output);

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

}  // namespace control_loops
}  // namespace frc971

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
