#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/controls/control_loop.h"
#include "aos/controls/polytope.h"
#include "aos/util/log_interval.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/gear.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainLoop : public aos::controls::ControlLoop<
                           ::frc971::control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      const DrivetrainConfig<double> &dt_config, ::aos::EventLoop *event_loop,
      LocalizerInterface *localizer,
      const ::std::string &name = ".frc971.control_loops.drivetrain_queue");

  int ControllerIndexFromGears();

 protected:
  // Executes one cycle of the control loop.
  void RunIteration(
      const ::frc971::control_loops::DrivetrainQueue::Goal *goal,
      const ::frc971::control_loops::DrivetrainQueue::Position *position,
      ::frc971::control_loops::DrivetrainQueue::Output *output,
      ::frc971::control_loops::DrivetrainQueue::Status *status) override;

  void Zero(::frc971::control_loops::DrivetrainQueue::Output *output) override;

  double last_gyro_rate_ = 0.0;

  const DrivetrainConfig<double> dt_config_;

  ::aos::Fetcher<LocalizerControl> localizer_control_fetcher_;
  LocalizerInterface *localizer_;

  StateFeedbackLoop<7, 2, 4> kf_;
  PolyDrivetrain<double> dt_openloop_;
  DrivetrainMotorsSS dt_closedloop_;
  SplineDrivetrain dt_spline_;
  LineFollowDrivetrain dt_line_follow_;
  ::aos::monotonic_clock::time_point last_gyro_time_ =
      ::aos::monotonic_clock::min_time;

  StateFeedbackLoop<2, 1, 1> down_estimator_;
  Eigen::Matrix<double, 1, 1> down_U_;

  // Current gears for each drive side.
  Gear left_gear_;
  Gear right_gear_;

  double last_left_voltage_ = 0;
  double last_right_voltage_ = 0;
  // The left/right voltages previous to last_*_voltage_.
  double last_last_left_voltage_ = 0;
  double last_last_right_voltage_ = 0;

  bool left_high_requested_;
  bool right_high_requested_;

  bool has_been_enabled_ = false;

  double last_accel_ = 0.0;

  // Last kalman filter state.
  ::Eigen::Matrix<double, 7, 1> last_state_ =
      ::Eigen::Matrix<double, 7, 1>::Zero();
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
