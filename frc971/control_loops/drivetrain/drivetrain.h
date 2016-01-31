#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "aos/common/util/log_interval.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainLoop : public aos::controls::ControlLoop<
                           ::frc971::control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(const DrivetrainConfig &dt_config,
      ::frc971::control_loops::DrivetrainQueue *my_drivetrain =
          &::frc971::control_loops::drivetrain_queue);

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
  double last_gyro_heading_ = 0.0;
  double last_gyro_rate_ = 0.0;

  const DrivetrainConfig dt_config_;

  PolyDrivetrain dt_openloop_{dt_config_};
  DrivetrainMotorsSS dt_closedloop_{dt_config_};
  StateFeedbackLoop<7, 2, 3> kf_;

  double last_left_voltage_ = 0;
  double last_right_voltage_ = 0;

  double integrated_kf_heading_ = 0;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
