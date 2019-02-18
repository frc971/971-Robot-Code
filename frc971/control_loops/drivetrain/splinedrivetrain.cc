#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

#include <iostream>

#include "Eigen/Dense"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

const int kMaxSplineConstraints = 6;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

SplineDrivetrain::SplineDrivetrain(const DrivetrainConfig<double> &dt_config)
    : dt_config_(dt_config),
      current_state_(::Eigen::Matrix<double, 2, 1>::Zero()) {}

void SplineDrivetrain::ScaleCapU(Eigen::Matrix<double, 2, 1> *U) {
  bool output_was_capped = ::std::abs((*U)(0, 0)) > 12.0 ||
                       ::std::abs((*U)(1, 0)) > 12.0;

  if (output_was_capped) {
    *U *= 12.0 / U->lpNorm<Eigen::Infinity>();
  }
}

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
    current_xva_ = current_trajectory_->FFAcceleration(0);
    current_xva_(1) = 0.0;
    current_state_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  }
}

// TODO(alex): Hold position when done following the spline.
// TODO(Austin): Compensate for voltage error.
void SplineDrivetrain::Update(bool enable,
                              const ::Eigen::Matrix<double, 5, 1> &state) {
  enable_ = enable;
  if (enable && current_trajectory_) {
    ::Eigen::Matrix<double, 2, 1> U_ff = ::Eigen::Matrix<double, 2, 1>::Zero();
    if (!current_trajectory_->is_at_end(current_state_)) {
      // TODO(alex): It takes about a cycle for the outputs to propagate to the
      // motors. Consider delaying the output by a cycle.
      U_ff = current_trajectory_->FFVoltage(current_xva_(0));
    }
    ::Eigen::Matrix<double, 2, 5> K =
        current_trajectory_->KForState(state, dt_config_.dt, Q, R);
    ::Eigen::Matrix<double, 5, 1> goal_state =
        current_trajectory_->GoalState(current_xva_(0), current_xva_(1));
    ::Eigen::Matrix<double, 5, 1> state_error = goal_state - state;
    ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;
    next_U_ = U_ff + U_fb;
    uncapped_U_ = next_U_;
    ScaleCapU(&next_U_);

    next_xva_ = current_trajectory_->GetNextXVA(dt_config_.dt, &current_state_);
  }
}

void SplineDrivetrain::SetOutput(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  if (!output) {
    return;
  }
  if (!current_trajectory_) {
    return;
  }
  if (current_spline_handle_ == current_spline_idx_) {
    if (!current_trajectory_->is_at_end(current_state_)) {
      output->left_voltage = next_U_(0);
      output->right_voltage = next_U_(1);
      current_xva_ = next_xva_;
    }
  }
}

void SplineDrivetrain::PopulateStatus(
    ::frc971::control_loops::DrivetrainQueue::Status *status) const {
  if (status && enable_) {
    status->uncapped_left_voltage = uncapped_U_(0);
    status->uncapped_right_voltage = uncapped_U_(1);
    status->robot_speed = current_xva_(1);
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
