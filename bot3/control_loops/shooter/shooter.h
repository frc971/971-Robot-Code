#ifndef BOT3_CONTROL_LOOPS_SHOOTER_H_
#define BOT3_CONTROL_LOOPS_SHOOTER_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"
#include "bot3/control_loops/shooter/shooter_motor_plant.h"

namespace bot3 {
namespace control_loops {

class ShooterMotor
    : public aos::control_loops::ControlLoop<control_loops::ShooterLoop> {
 public:
  explicit ShooterMotor(
      control_loops::ShooterLoop *my_shooter = &control_loops::shooter);

  // Control loop time step.
  static const double dt;

  // Maximum speed of the shooter wheel which the encoder is rated for in
  // rad/sec.
  static const double kMaxSpeed;

 protected:
  virtual void RunIteration(
      const control_loops::ShooterLoop::Goal *goal,
      const control_loops::ShooterLoop::Position *position,
      ::aos::control_loops::Output *output,
      control_loops::ShooterLoop::Status *status);

 private:
  // The state feedback control loop to talk to.
  ::std::unique_ptr<StateFeedbackLoop<1, 1, 1>> loop_;

  double average_velocity_;

  // For making sure it keeps spinning if we're shooting.
  double last_velocity_goal_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace bot3

#endif // BOT3_CONTROL_LOOPS_SHOOTER_H_
