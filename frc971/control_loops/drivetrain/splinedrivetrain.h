#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_

#include "Eigen/Dense"

#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/drivetrain/trajectory.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class SplineDrivetrain {
 public:
  SplineDrivetrain(const DrivetrainConfig<double> &dt_config);

  void SetGoal(const ::frc971::control_loops::DrivetrainQueue::Goal &goal);

  void Update(bool enabled);

  void SetOutput(
      ::frc971::control_loops::DrivetrainQueue::Output *output);
  // TODO(alex): What status do we need?
  void PopulateStatus(
      ::frc971::control_loops::DrivetrainQueue::Status *status) const;
 private:
  const DrivetrainConfig<double> dt_config_;

  uint32_t current_spline_handle_; // Current spline told to excecute.
  uint32_t current_spline_idx_;  // Current excecuting spline.
  ::std::unique_ptr<DistanceSpline> distance_spline_;
  ::std::unique_ptr<Trajectory> current_trajectory_;
  ::Eigen::Matrix<double, 3, 1> current_xva_, next_xva_;
  ::Eigen::Matrix<double, 2, 1> current_state_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
