#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_

#include "aos/common/controls/polytope.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class PolyDrivetrain {
 public:
  enum Gear { HIGH, LOW, SHIFTING_UP, SHIFTING_DOWN };

  PolyDrivetrain(const DrivetrainConfig &dt_config);

  int controller_index() const { return loop_->controller_index(); }

  bool IsInGear(Gear gear) { return gear == LOW || gear == HIGH; }

  // Computes the speed of the motor given the hall effect position and the
  // speed of the robot.
  double MotorSpeed(const constants::ShifterHallEffect &hall_effect,
                    double shifter_position, double velocity, Gear gear);

  // Computes the states of the shifters for the left and right drivetrain sides
  // given a requested state.
  void UpdateGears(Gear requested_gear);

  // Computes the next state of a shifter given the current state and the
  // requested state.
  Gear UpdateSingleGear(Gear requested_gear, Gear current_gear);

  void SetGoal(double wheel, double throttle, bool quickturn, bool highgear);

  void SetPosition(
      const ::frc971::control_loops::DrivetrainQueue::Position *position);

  double FilterVelocity(double throttle);

  double MaxVelocity();

  void Update();

  void SendMotors(::frc971::control_loops::DrivetrainQueue::Output *output);

 private:
  StateFeedbackLoop<7, 2, 3> kf_;

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
  ::frc971::control_loops::DrivetrainQueue::Position last_position_;
  ::frc971::control_loops::DrivetrainQueue::Position position_;
  int counter_;
  DrivetrainConfig dt_config_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
