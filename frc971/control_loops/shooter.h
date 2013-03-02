#ifndef FRC971_CONTROL_LOOPS_SHOOTER_H_
#define FRC971_CONTROL_LOOPS_SHOOTER_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/shooter_motor.q.h"
#include "frc971/control_loops/shooter_motor_plant.h"

namespace frc971 {
namespace control_loops {

class ShooterMotor
    : public aos::control_loops::ControlLoop<control_loops::ShooterLoop> {
 public:
  explicit ShooterMotor(
      control_loops::ShooterLoop *my_shooter = &control_loops::shooter);

 protected:
  virtual void RunIteration(
      const control_loops::ShooterLoop::Goal *goal,
      const control_loops::ShooterLoop::Position *position,
      ::aos::control_loops::Output *output,
      control_loops::ShooterLoop::Status *status);
 private:
  // Timestep of the control loop (in seconds)
  static constexpr double dt = 0.01;

  // Maximum speed of the shooter wheel which the encoder is rated for.
  // 10000 rpm * (2 * M_PI radians / rotation) / (60 sec / min) * 15 / 34
  static constexpr double max_speed = 461.998919646;

  // The state feedback control loop to talk to.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> loop_;

  // History array and stuff for determining average velocity and whether
  // we are ready to shoot.
  static const int kHistoryLength = 10;
  double history[kHistoryLength];
  ptrdiff_t history_position_;
  double average_velocity_;

  double position_goal_;

  // time_ is used for recording data so that we have a time.
  double time_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif // FRC971_CONTROL_LOOPS_SHOOTER_H_
