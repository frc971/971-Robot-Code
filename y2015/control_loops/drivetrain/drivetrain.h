#ifndef Y2015_CONTROL_LOOPS_DRIVETRAIN_H_
#define Y2015_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "y2015/control_loops/drivetrain/drivetrain.q.h"
#include "aos/common/util/log_interval.h"

namespace frc971 {
namespace control_loops {

class DrivetrainLoop
    : public aos::controls::ControlLoop<control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(control_loops::DrivetrainQueue *my_drivetrain =
                              &control_loops::drivetrain_queue)
      : aos::controls::ControlLoop<control_loops::DrivetrainQueue>(
            my_drivetrain) {
    ::aos::controls::HPolytope<0>::Init();
  }

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const control_loops::DrivetrainQueue::Goal *goal,
      const control_loops::DrivetrainQueue::Position *position,
      control_loops::DrivetrainQueue::Output *output,
      control_loops::DrivetrainQueue::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");
};

}  // namespace control_loops
}  // namespace frc971

#endif  // Y2015_CONTROL_LOOPS_DRIVETRAIN_H_
