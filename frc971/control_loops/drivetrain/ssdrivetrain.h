#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_

#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/matrix_logging.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainMotorsSS {
 public:
  class LimitedDrivetrainLoop : public StateFeedbackLoop<4, 2, 2> {
   public:
    LimitedDrivetrainLoop(StateFeedbackLoop<4, 2, 2> &&loop);

    bool output_was_capped() const {
      return output_was_capped_;
    }

   private:
    void CapU() override;

    // Reprsents +/- full power on each motor in U-space, aka the square from
    // (-12, -12) to (12, 12).
    const ::aos::controls::HPolytope<2> U_poly_;

    // multiplying by T converts [left_error, right_error] to
    // [left_right_error_difference, total_distance_error].
    Eigen::Matrix<double, 2, 2> T_, T_inverse_;

    bool output_was_capped_ = false;
  };

  DrivetrainMotorsSS(const DrivetrainConfig &dt_config);

  void SetGoal(double left, double left_velocity, double right,
               double right_velocity);

  void SetRawPosition(double left, double right);

  void SetPosition(double left, double right, double gyro);

  void SetExternalMotors(double left_voltage, double right_voltage);

  void Update(bool stop_motors, bool enable_control_loop);

  double GetEstimatedRobotSpeed() const;

  double GetEstimatedLeftEncoder() const {
    return loop_->X_hat(0, 0);
  }

  double left_velocity() const { return loop_->X_hat(1, 0); }
  double right_velocity() const { return loop_->X_hat(3, 0); }

  double GetEstimatedRightEncoder() const {
    return loop_->X_hat(2, 0);
  }

  bool OutputWasCapped() const {
    return loop_->output_was_capped();
  }

  void SendMotors(
      ::frc971::control_loops::DrivetrainQueue::Output *output) const;

  const LimitedDrivetrainLoop &loop() const { return *loop_; }

 private:
  ::std::unique_ptr<LimitedDrivetrainLoop> loop_;

  double filtered_offset_;
  double gyro_;
  double left_goal_;
  double right_goal_;
  double raw_left_;
  double raw_right_;

  const DrivetrainConfig dt_config_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
