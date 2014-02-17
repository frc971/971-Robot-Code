#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/controls/polytope.h"
#include "aos/common/control_loop/ControlLoop.h"
#include "aos/controls/polytope.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "aos/common/util/log_interval.h"

namespace frc971 {
namespace control_loops {

Eigen::Matrix<double, 2, 1> CoerceGoal(aos::controls::HPolytope<2> &region,
                                       const Eigen::Matrix<double, 1, 2> &K,
                                       double w,
                                       const Eigen::Matrix<double, 2, 1> &R);

class DrivetrainLoop
    : public aos::control_loops::ControlLoop<control_loops::Drivetrain, true, false> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      control_loops::Drivetrain *my_drivetrain = &control_loops::drivetrain)
      : aos::control_loops::ControlLoop<control_loops::Drivetrain, true, false>(
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
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
