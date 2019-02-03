#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

#include "Eigen/Dense"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

const int kMaxSplineConstraints = 6;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

SplineDrivetrain::SplineDrivetrain(const DrivetrainConfig<double> &dt_config)
    : dt_config_(dt_config) {}

// TODO(alex): put in another thread to avoid malloc in RT.
void SplineDrivetrain::SetGoal(
    const ::frc971::control_loops::DrivetrainQueue::Goal &goal) {
  current_spline_handle_ = goal.spline_handle;
  const ::frc971::MultiSpline &multispline = goal.spline;
  if (multispline.spline_idx) {
    current_spline_idx_ = multispline.spline_idx;
    auto x = multispline.spline_x;
    auto y = multispline.spline_y;
    ::std::vector<Spline> splines = ::std::vector<Spline>();
    for (int i = 0; i < multispline.spline_count; ++i) {
      ::Eigen::Matrix<double, 2, 6> points =
          ::Eigen::Matrix<double, 2, 6>::Zero();
      for (int j = 0; j < 6; ++j) {
        points(0, j) = x[i * 5 + j];
        points(1, j) = y[i * 5 + j];
      }
      splines.emplace_back(Spline(points));
    }

    distance_spline_ = ::std::unique_ptr<DistanceSpline>(
        new DistanceSpline(::std::move(splines)));

    current_trajectory_ = ::std::unique_ptr<Trajectory>(
        new Trajectory(distance_spline_.get(), dt_config_));

    for (int i = 0; i < kMaxSplineConstraints; ++i) {
      const ::frc971::Constraint &constraint = multispline.constraints[i];
      switch (constraint.constraint_type) {
        case 0:
          break;
        case 1:
          current_trajectory_->set_longitudal_acceleration(constraint.value);
          break;
        case 2:
          current_trajectory_->set_lateral_acceleration(constraint.value);
          break;
        case 3:
          current_trajectory_->set_voltage_limit(constraint.value);
          break;
        case 4:
          current_trajectory_->LimitVelocity(constraint.start_distance,
                                             constraint.end_distance,
                                             constraint.value);
          break;
      }
    }

    current_trajectory_->Plan();
    current_xva_plan_ = current_trajectory_->PlanXVA(dt_config_.dt);
  }
}

// TODO(alex): Handle drift.
void SplineDrivetrain::SetOutput(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  if (!output) {
    return;
  }
  if (current_spline_handle_ == current_spline_idx_) {
    if (current_xva_idx_ < current_xva_plan_.size()) {
      double current_distance = current_xva_plan_[current_xva_idx_](0);
      ::Eigen::Matrix<double, 2, 1> FFVoltage =
          current_trajectory_->FFVoltage(current_distance);
      output->left_voltage = FFVoltage(0);
      output->right_voltage = FFVoltage(1);
      ++current_xva_idx_;
    }
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
