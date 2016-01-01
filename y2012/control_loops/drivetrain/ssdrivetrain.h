#ifndef Y2014_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
#define Y2014_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_

#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/matrix_logging.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2012/control_loops/drivetrain/drivetrain.q.h"

namespace y2012 {
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

    const ::aos::controls::HPolytope<2> U_Poly_;
    Eigen::Matrix<double, 2, 2> T, T_inverse;
    bool output_was_capped_ = false;;
  };

  DrivetrainMotorsSS();

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
      ::y2012::control_loops::DrivetrainQueue::Output *output) const;

  const LimitedDrivetrainLoop &loop() const { return *loop_; }

 private:
  ::std::unique_ptr<LimitedDrivetrainLoop> loop_;

  double filtered_offset_;
  double gyro_;
  double left_goal_;
  double right_goal_;
  double raw_left_;
  double raw_right_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2012

#endif  // Y2014_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
