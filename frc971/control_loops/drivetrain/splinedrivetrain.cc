#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

#include "Eigen/Dense"

#include "aos/realtime.h"
#include "aos/util/math.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"

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
    while (goal_.spline_idx == future_spline_idx_) {
      AOS_CHECK(!new_goal_.Wait());
      if (!run_) {
        return;
      }
    }
    past_distance_spline_.reset();
    past_trajectory_.reset();

    plan_state_ = PlanningState::BUILDING_TRAJECTORY;
    future_spline_idx_ = goal_.spline_idx;
    planning_spline_idx_ = future_spline_idx_;
    const MultiSpline &multispline = goal_.spline;
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

    future_drive_spline_backwards_ = goal_.drive_spline_backwards;

    future_distance_spline_ = ::std::unique_ptr<DistanceSpline>(
        new DistanceSpline(::std::move(splines)));

    future_trajectory_ = ::std::unique_ptr<Trajectory>(
        new Trajectory(future_distance_spline_.get(), dt_config_));

    for (size_t i = 0; i < multispline.constraints.size(); ++i) {
      const ::frc971::ConstraintT &constraint = multispline.constraints[i];
      switch (constraint.constraint_type) {
        case frc971::ConstraintType::CONSTRAINT_TYPE_UNDEFINED:
          break;
        case frc971::ConstraintType::LONGITUDINAL_ACCELERATION:
          future_trajectory_->set_longitudinal_acceleration(constraint.value);
          break;
        case frc971::ConstraintType::LATERAL_ACCELERATION:
          future_trajectory_->set_lateral_acceleration(constraint.value);
          break;
        case frc971::ConstraintType::VOLTAGE:
          future_trajectory_->set_voltage_limit(constraint.value);
          break;
        case frc971::ConstraintType::VELOCITY:
          future_trajectory_->LimitVelocity(constraint.start_distance,
                                            constraint.end_distance,
                                            constraint.value);
          break;
      }
    }
    plan_state_ = PlanningState::PLANNING_TRAJECTORY;

    future_trajectory_->Plan();
    plan_state_ = PlanningState::PLANNED;
  }
}

void SplineDrivetrain::SetGoal(
    const ::frc971::control_loops::drivetrain::Goal *goal) {
  current_spline_handle_ = goal->spline_handle();

  // If told to stop, set the executing spline to an invalid index and clear out
  // its plan:
  if (current_spline_handle_ == 0 &&
      (goal->spline() == nullptr ||
       current_spline_idx_ != CHECK_NOTNULL(goal->spline())->spline_idx())) {
    current_spline_idx_ = -1;
  }

  ::aos::Mutex::State mutex_state = mutex_.TryLock();
  if (mutex_state == ::aos::Mutex::State::kLocked) {
    if (goal->spline() != nullptr && goal->spline()->spline_idx() &&
        future_spline_idx_ != goal->spline()->spline_idx()) {
      CHECK_NOTNULL(goal->spline());
      goal_.spline_idx = goal->spline()->spline_idx();
      goal_.drive_spline_backwards = goal->spline()->drive_spline_backwards();

      const frc971::MultiSpline *multispline = goal->spline()->spline();
      CHECK_NOTNULL(multispline);

      goal_.spline.spline_count = multispline->spline_count();

      CHECK_EQ(multispline->spline_x()->size(),
               static_cast<size_t>(goal_.spline.spline_count * 5 + 1));
      CHECK_EQ(multispline->spline_y()->size(),
               static_cast<size_t>(goal_.spline.spline_count * 5 + 1));

      std::copy(multispline->spline_x()->begin(),
                multispline->spline_x()->end(), goal_.spline.spline_x.begin());
      std::copy(multispline->spline_y()->begin(),
                multispline->spline_y()->end(), goal_.spline.spline_y.begin());

      for (size_t i = 0; i < 6; ++i) {
        if (multispline->constraints() != nullptr &&
            i < multispline->constraints()->size()) {
          multispline->constraints()->Get(i)->UnPackTo(
              &goal_.spline.constraints[i]);
        } else {
          goal_.spline.constraints[i].constraint_type =
              ConstraintType::CONSTRAINT_TYPE_UNDEFINED;
        }
      }

      new_goal_.Broadcast();
      if (current_spline_handle_ != current_spline_idx_) {
        // If we aren't going to actively execute the current spline, evict it's
        // plan.
        past_trajectory_ = std::move(current_trajectory_);
        past_distance_spline_ = std::move(current_distance_spline_);
      }
    }
    // If you never started executing the previous spline, you're screwed.
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
      current_drive_spline_backwards_ = future_drive_spline_backwards_;
      current_spline_idx_ = future_spline_idx_;

      // Reset internal state to a trajectory start position.
      current_xva_ = current_trajectory_->FFAcceleration(0);
      current_xva_(1) = 0.0;
      has_started_execution_ = false;
    }
    mutex_.Unlock();
  } else {
    VLOG(1) << "Failed to acquire trajectory lock.";
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
    if (current_drive_spline_backwards_) {
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
    ::frc971::control_loops::drivetrain::OutputT *output) {
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
  drivetrain::Status::Builder *builder) const {
  if (builder && enable_) {
    builder->add_uncapped_left_voltage(uncapped_U_(0));
    builder->add_uncapped_right_voltage(uncapped_U_(1));
    builder->add_robot_speed(current_xva_(1));
    builder->add_output_was_capped(output_was_capped_);
  }
}

flatbuffers::Offset<TrajectoryLogging> SplineDrivetrain::MakeTrajectoryLogging(
    flatbuffers::FlatBufferBuilder *builder) const {
  drivetrain::TrajectoryLogging::Builder trajectory_logging_builder(*builder);
  if (current_distance_spline_) {
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    trajectory_logging_builder.add_x(goal_state(0));
    trajectory_logging_builder.add_y(goal_state(1));
    trajectory_logging_builder.add_theta(::aos::math::NormalizeAngle(
        goal_state(2) + (current_drive_spline_backwards_ ? M_PI : 0.0)));
    trajectory_logging_builder.add_left_velocity(goal_state(3));
    trajectory_logging_builder.add_right_velocity(goal_state(4));
  }
  trajectory_logging_builder.add_planning_state(plan_state_.load());
  trajectory_logging_builder.add_is_executing(!IsAtEnd() &&
                                              has_started_execution_);
  trajectory_logging_builder.add_is_executed((current_spline_idx_ != -1) &&
                                             IsAtEnd());
  trajectory_logging_builder.add_goal_spline_handle(current_spline_handle_);
  trajectory_logging_builder.add_current_spline_idx(current_spline_idx_);
  trajectory_logging_builder.add_distance_remaining(
      current_trajectory_ ? current_trajectory_->length() - current_xva_.x()
                          : 0.0);

  int32_t planning_spline_idx = planning_spline_idx_;
  if (current_spline_idx_ == planning_spline_idx) {
    trajectory_logging_builder.add_planning_spline_idx(0);
  } else {
    trajectory_logging_builder.add_planning_spline_idx(planning_spline_idx_);
  }
  return trajectory_logging_builder.Finish();
}

flatbuffers::Offset<TrajectoryLogging> SplineDrivetrain::MakeTrajectoryLogging(
    aos::Sender<drivetrain::Status>::Builder *builder) const {
  return MakeTrajectoryLogging(builder->fbb());
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
