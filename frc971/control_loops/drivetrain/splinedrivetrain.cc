#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

#include "Eigen/Dense"

#include "aos/init.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

SplineDrivetrain::SplineDrivetrain(const DrivetrainConfig<double> &dt_config)
    : dt_config_(dt_config), new_goal_(&mutex_) {
  worker_thread_ = std::thread(&SplineDrivetrain::ComputeTrajectory, this);
}

void SplineDrivetrain::ScaleCapU(Eigen::Matrix<double, 2, 1> *U) {
  output_was_capped_ =
      ::std::abs((*U)(0, 0)) > 12.0 || ::std::abs((*U)(1, 0)) > 12.0;

  if (output_was_capped_) {
    *U *= 12.0 / U->lpNorm<Eigen::Infinity>();
  }
}

void SplineDrivetrain::ComputeTrajectory() {
  ::aos::SetCurrentThreadRealtimePriority(1);

  ::aos::MutexLocker locker(&mutex_);
  while (run_) {
    while (goal_.spline.spline_idx == future_spline_idx_) {
      CHECK(!new_goal_.Wait());
      if (!run_) {
        return;
      }
    }
    past_distance_spline_.reset();
    past_trajectory_.reset();

    plan_state_ = PlanState::kBuildingTrajectory;
    const ::frc971::MultiSpline &multispline = goal_.spline;
    future_spline_idx_ = multispline.spline_idx;
    planning_spline_idx_ = future_spline_idx_;
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

    future_distance_spline_ = ::std::unique_ptr<DistanceSpline>(
        new DistanceSpline(::std::move(splines)));

    future_trajectory_ = ::std::unique_ptr<Trajectory>(
        new Trajectory(future_distance_spline_.get(), dt_config_));

    for (size_t i = 0; i < multispline.constraints.size(); ++i) {
      const ::frc971::Constraint &constraint = multispline.constraints[i];
      switch (constraint.constraint_type) {
        case 0:
          break;
        case 1:
          future_trajectory_->set_longitudal_acceleration(constraint.value);
          break;
        case 2:
          future_trajectory_->set_lateral_acceleration(constraint.value);
          break;
        case 3:
          future_trajectory_->set_voltage_limit(constraint.value);
          break;
        case 4:
          future_trajectory_->LimitVelocity(constraint.start_distance,
                                            constraint.end_distance,
                                            constraint.value);
          break;
      }
    }
    plan_state_ = PlanState::kPlanningTrajectory;

    future_trajectory_->Plan();
    plan_state_ = PlanState::kPlannedTrajectory;
  }
}

void SplineDrivetrain::SetGoal(
    const ::frc971::control_loops::DrivetrainQueue::Goal &goal) {
  current_spline_handle_ = goal.spline_handle;
  // If told to stop, set the executing spline to an invalid index.
  if (current_spline_handle_ == 0 && has_started_execution_) {
    current_spline_idx_ = -1;
  }

  ::aos::Mutex::State mutex_state = mutex_.TryLock();
  if (mutex_state == ::aos::Mutex::State::kLocked) {
    drive_spline_backwards_ = goal_.drive_spline_backwards;
    if (goal.spline.spline_idx && future_spline_idx_ != goal.spline.spline_idx) {
      goal_ = goal;
      new_goal_.Broadcast();
    }
    if (future_trajectory_ &&
        (!current_trajectory_ ||
         current_trajectory_->is_at_end(current_xva_.block<2, 1>(0, 0)) ||
         current_spline_idx_ == -1)) {
      // Move current data to other variables to be cleared by worker.
      past_trajectory_ = std::move(current_trajectory_);
      past_distance_spline_ = std::move(current_distance_spline_);

      // Move the computed data to be executed.
      current_trajectory_ = std::move(future_trajectory_);
      current_distance_spline_ = std::move(future_distance_spline_);
      current_spline_idx_ = future_spline_idx_;

      // Reset internal state to a trajectory start position.
      current_xva_ = current_trajectory_->FFAcceleration(0);
      current_xva_(1) = 0.0;
      has_started_execution_ = false;
    }
    mutex_.Unlock();
  }
}

// TODO(alex): Hold position when done following the spline.
// TODO(Austin): Compensate for voltage error.
void SplineDrivetrain::Update(bool enable, const ::Eigen::Matrix<double, 5, 1> &state) {
  next_U_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  enable_ = enable;
  if (enable && current_trajectory_) {
    ::Eigen::Matrix<double, 2, 1> U_ff = ::Eigen::Matrix<double, 2, 1>::Zero();
    if (!IsAtEnd() &&
        current_spline_handle_ == current_spline_idx_) {
      has_started_execution_ = true;
      // TODO(alex): It takes about a cycle for the outputs to propagate to the
      // motors. Consider delaying the output by a cycle.
      U_ff = current_trajectory_->FFVoltage(current_xva_(0));
    }

    ::Eigen::Matrix<double, 2, 5> K =
        current_trajectory_->KForState(state, dt_config_.dt, Q, R);
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    if (drive_spline_backwards_) {
      ::Eigen::Matrix<double, 2, 1> swapU(U_ff(1, 0), U_ff(0, 0));
      U_ff = -swapU;
      goal_state(2, 0) += M_PI;
      double left_goal = goal_state(3, 0);
      double right_goal = goal_state(4, 0);
      goal_state(3, 0) = -right_goal;
      goal_state(4, 0) = -left_goal;
    }
    ::Eigen::Matrix<double, 5, 1> state_error = goal_state - state;
    state_error(2, 0) = ::aos::math::NormalizeAngle(state_error(2, 0));
    ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;

    ::Eigen::Matrix<double, 2, 1> xv_state = current_xva_.block<2,1>(0,0);
    next_xva_ = current_trajectory_->GetNextXVA(dt_config_.dt, &xv_state);
    next_U_ = U_ff + U_fb;
    uncapped_U_ = next_U_;
    ScaleCapU(&next_U_);
  }
}

void SplineDrivetrain::SetOutput(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  if (!output) {
    return;
  }
  if (current_spline_handle_ == current_spline_idx_) {
    if (!IsAtEnd()) {
      output->left_voltage = next_U_(0);
      output->right_voltage = next_U_(1);
      current_xva_ = next_xva_;
    }
  }
  output->left_voltage = next_U_(0);
  output->right_voltage = next_U_(1);
}

void SplineDrivetrain::PopulateStatus(
    ::frc971::control_loops::DrivetrainQueue::Status *status) const {
  if (status && enable_) {
    status->uncapped_left_voltage = uncapped_U_(0);
    status->uncapped_right_voltage = uncapped_U_(1);
    status->robot_speed = current_xva_(1);
    status->output_was_capped = output_was_capped_;
  }

  if (status) {
    if (current_distance_spline_) {
      ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
      status->trajectory_logging.x = goal_state(0);
      status->trajectory_logging.y = goal_state(1);
      status->trajectory_logging.theta = goal_state(2);
      status->trajectory_logging.left_velocity = goal_state(3);
      status->trajectory_logging.right_velocity = goal_state(4);
    }
    status->trajectory_logging.planning_state = static_cast<int8_t>(plan_state_.load());
    status->trajectory_logging.is_executing = !IsAtEnd() && has_started_execution_;
    status->trajectory_logging.goal_spline_handle = current_spline_handle_;
    status->trajectory_logging.current_spline_idx = current_spline_idx_;

    int32_t planning_spline_idx = planning_spline_idx_;
    if (current_spline_idx_ == planning_spline_idx) {
      status->trajectory_logging.planning_spline_idx = 0;
    } else {
      status->trajectory_logging.planning_spline_idx = planning_spline_idx_;
    }
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
