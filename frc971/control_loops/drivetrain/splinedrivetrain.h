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

  void Update(bool enabled, const ::Eigen::Matrix<double, 5, 1> &state);

  void SetOutput(
      ::frc971::control_loops::DrivetrainQueue::Output *output);
  // TODO(alex): What status do we need?
  void PopulateStatus(
      ::frc971::control_loops::DrivetrainQueue::Status *status) const;

  // Accessor for the current goal state, pretty much only present for debugging
  // purposes.
  Eigen::Matrix<double, 5, 1> CurrentGoalState() const {
    return current_trajectory_->GoalState(current_xva_(0), current_xva_(1));
  }

  bool IsAtEnd() const {
    return current_trajectory_->is_at_end(current_state_);
  }
 private:
  void ScaleCapU(Eigen::Matrix<double, 2, 1> *U);

  const DrivetrainConfig<double> dt_config_;

  uint32_t current_spline_handle_; // Current spline told to excecute.
  uint32_t current_spline_idx_;  // Current excecuting spline.
  ::std::unique_ptr<DistanceSpline> distance_spline_;
  ::std::unique_ptr<Trajectory> current_trajectory_;
  ::Eigen::Matrix<double, 3, 1> current_xva_, next_xva_;
  ::Eigen::Matrix<double, 2, 1> current_state_;
  ::Eigen::Matrix<double, 2, 1> next_U_;
  ::Eigen::Matrix<double, 2, 1> uncapped_U_;

  bool enable_;

  // TODO(alex): pull this out of dt_config.
  const ::Eigen::DiagonalMatrix<double, 5> Q =
      (::Eigen::DiagonalMatrix<double, 5>().diagonal()
           << 1.0 / ::std::pow(0.05, 2),
       1.0 / ::std::pow(0.05, 2), 1.0 / ::std::pow(0.2, 2),
       1.0 / ::std::pow(0.5, 2), 1.0 / ::std::pow(0.5, 2))
          .finished()
          .asDiagonal();
  const ::Eigen::DiagonalMatrix<double, 2> R =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
