#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/control_loops/drivetrain/polydrivetrain.h"
#include "y2014/control_loops/drivetrain/ssdrivetrain.h"
#include "aos/common/util/log_interval.h"

namespace y2014 {
namespace control_loops {
namespace drivetrain {

class DrivetrainLoop
    : public aos::controls::ControlLoop<::frc971::control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      ::frc971::control_loops::DrivetrainQueue *my_drivetrain =
          &::frc971::control_loops::drivetrain_queue)
      : aos::controls::ControlLoop<control_loops::DrivetrainQueue>(
            my_drivetrain) {
    ::aos::controls::HPolytope<0>::Init();
  }

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const ::frc971::control_loops::DrivetrainQueue::Goal *goal,
      const ::frc971::control_loops::DrivetrainQueue::Position *position,
      ::frc971::control_loops::DrivetrainQueue::Output *output,
      ::frc971::control_loops::DrivetrainQueue::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");

  PolyDrivetrain dt_openloop_;
  DrivetrainMotorsSS dt_closedloop_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_H_
