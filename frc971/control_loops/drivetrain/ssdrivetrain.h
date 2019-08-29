#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_

#include "aos/commonmath.h"
#include "aos/controls/control_loop.h"
#include "aos/controls/polytope.h"
#include "aos/util/trapezoid_profile.h"

#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainMotorsSS {
 public:
  DrivetrainMotorsSS(const DrivetrainConfig<double> &dt_config,
                     StateFeedbackLoop<7, 2, 4> *kf,
                     LocalizerInterface *localizer);

  void SetGoal(const ::frc971::control_loops::drivetrain::Goal *goal);

  // Computes the power to send out as part of the controller.  Should be called
  // when disabled (with enable_control_loop false) so the profiles get computed
  // correctly.
  // enable_control_loop includes the actual enable bit and if the loop will go
  // out to hw.
  void Update(bool enable_control_loop);

  bool output_was_capped() const { return output_was_capped_; }

  void SetOutput(::frc971::control_loops::drivetrain::OutputT *output) const;
  void PopulateStatus(
      ::frc971::control_loops::drivetrain::StatusBuilder *builder) const;

 private:
  void PolyCapU(Eigen::Matrix<double, 2, 1> *U);
  void ScaleCapU(Eigen::Matrix<double, 2, 1> *U);

  const DrivetrainConfig<double> dt_config_;
  StateFeedbackLoop<7, 2, 4> *kf_;
  Eigen::Matrix<double, 7, 1> unprofiled_goal_;

  double last_gyro_to_wheel_offset_ = 0;

  constexpr static double kMaxVoltage = 12.0;
  double max_voltage_ = kMaxVoltage;

  // Reprsents +/- full power on each motor in U-space, aka the square from
  // (-12, -12) to (12, 12).
  const ::aos::controls::HVPolytope<2, 4, 4> U_poly_;

  // multiplying by T converts [left_error, right_error] to
  // [left_right_error_difference, total_distance_error].
  Eigen::Matrix<double, 2, 2> T_, T_inverse_;

  aos::util::TrapezoidProfile linear_profile_, angular_profile_;

  bool output_was_capped_ = false;

  bool use_profile_ = false;

  LocalizerInterface *localizer_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
