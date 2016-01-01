#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_

#include "aos/common/controls/polytope.h"

#include "y2012/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2012/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

namespace y2012 {
namespace control_loops {
namespace drivetrain {

class PolyDrivetrain {
 public:
  enum Gear { HIGH, LOW, SHIFTING_UP, SHIFTING_DOWN };
  // Stall Torque in N m
  static constexpr double kStallTorque = drivetrain::kStallTorque;
  // Stall Current in Amps
  static constexpr double kStallCurrent = drivetrain::kStallCurrent;
  // Free Speed in RPM. Used number from last year.
  static constexpr double kFreeSpeed = drivetrain::kFreeSpeedRPM;
  // Free Current in Amps
  static constexpr double kFreeCurrent = drivetrain::kFreeCurrent;
  static constexpr double kWheelRadius = drivetrain::kWheelRadius;
  // Resistance of the motor, divided by the number of motors per side.
  static constexpr double kR = drivetrain::kR;
  // Motor velocity constant
  static constexpr double Kv = drivetrain::kV;

  // Torque constant
  static constexpr double Kt = drivetrain::kT;

  static constexpr double kLowGearRatio = 15.0 / 60.0 * 15.0 / 50.0;
  static constexpr double kHighGearRatio = 30.0 / 45.0 * 15.0 / 50.0;

  PolyDrivetrain();

  int controller_index() const { return loop_->controller_index(); }

  static bool IsInGear(Gear gear) { return gear == LOW || gear == HIGH; }

  static double MotorSpeed(bool high_gear, double velocity);

  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear);

  void SetPosition(
      const ::y2012::control_loops::DrivetrainQueue::Position *position);

  double FilterVelocity(double throttle);

  double MaxVelocity();

  void Update();

  void SendMotors(::y2012::control_loops::DrivetrainQueue::Output *output);

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
  ::y2012::control_loops::DrivetrainQueue::Position last_position_;
  ::y2012::control_loops::DrivetrainQueue::Position position_;
  int counter_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2012

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
