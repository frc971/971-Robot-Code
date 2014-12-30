#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/util/log_interval.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"

namespace bot3 {
namespace control_loops {

class DrivetrainLoop
    : public aos::controls::ControlLoop<control_loops::Drivetrain, true, false> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at bot3::control_loops::drivetrain
  explicit DrivetrainLoop(
      control_loops::Drivetrain *my_drivetrain = &control_loops::drivetrain)
      : aos::controls::ControlLoop<control_loops::Drivetrain, true, false>(
          my_drivetrain) {
    ::aos::controls::HPolytope<0>::Init();
  }

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const control_loops::Drivetrain::Goal *goal,
      const control_loops::Drivetrain::Position *position,
      control_loops::Drivetrain::Output *output,
      control_loops::Drivetrain::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");
};

}  // namespace control_loops
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
