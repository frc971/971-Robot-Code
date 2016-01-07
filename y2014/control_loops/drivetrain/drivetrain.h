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

class DrivetrainLoop : public aos::controls::ControlLoop<
                           ::y2014::control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at y2014::control_loops::drivetrain
  explicit DrivetrainLoop(
      ::y2014::control_loops::DrivetrainQueue *my_drivetrain =
          &::y2014::control_loops::drivetrain_queue);

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const ::y2014::control_loops::DrivetrainQueue::Goal *goal,
      const ::y2014::control_loops::DrivetrainQueue::Position *position,
      ::y2014::control_loops::DrivetrainQueue::Output *output,
      ::y2014::control_loops::DrivetrainQueue::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");
  double last_gyro_heading_ = 0.0;
  double last_gyro_rate_ = 0.0;

  StateFeedbackLoop<7, 2, 3> kf_;
  PolyDrivetrain dt_openloop_;
  DrivetrainMotorsSS dt_closedloop_;

  double last_left_voltage_ = 0;
  double last_right_voltage_ = 0;

  double integrated_kf_heading_ = 0;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_H_
