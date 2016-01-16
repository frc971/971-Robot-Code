#ifndef Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
#define Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016/control_loops/shooter/shooter_plant.h"
#include "y2016/control_loops/shooter/shooter.q.h"

namespace y2016 {
namespace control_loops {

class Shooter
    : public ::aos::controls::ControlLoop<control_loops::ShooterQueue> {
 public:
  explicit Shooter(
      control_loops::ShooterQueue *my_shooter = &control_loops::shooter_queue);

  // Control loop time step.
  static const double dt;

  // Maximum speed of the shooter wheel which the encoder is rated for in
  // rad/sec.
  static const double kMaxSpeed;

 protected:
  virtual void RunIteration(
      const control_loops::ShooterQueue::Goal *goal,
      const control_loops::ShooterQueue::Position *position,
      ::aos::control_loops::Output *output,
      control_loops::ShooterQueue::Status *status);

 private:
  // The state feedback control loop to talk to.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> loop_;

  // History array and stuff for determining average velocity and whether
  // we are ready to shoot.
  static const int kHistoryLength = 5;
  double history_[kHistoryLength];
  ptrdiff_t history_position_;
  double average_velocity_;

  double position_goal_;
  double last_position_;

  // For making sure it keeps spinning if we're shooting.
  double last_velocity_goal_;

  DISALLOW_COPY_AND_ASSIGN(Shooter);
};

}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
