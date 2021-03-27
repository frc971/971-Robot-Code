#include "frc971/control_loops/drivetrain/splinedrivetrain.h"

#include "Eigen/Dense"

#include "aos/json_to_flatbuffer.h"

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
    : dt_config_(dt_config),
      current_xva_(Eigen::Vector3d::Zero()),
      next_xva_(Eigen::Vector3d::Zero()),
      next_U_(Eigen::Vector2d::Zero()) {}

void SplineDrivetrain::ScaleCapU(Eigen::Matrix<double, 2, 1> *U) {
  output_was_capped_ =
      ::std::abs((*U)(0, 0)) > 12.0 || ::std::abs((*U)(1, 0)) > 12.0;

  if (output_was_capped_) {
    *U *= 12.0 / U->lpNorm<Eigen::Infinity>();
  }
}

void SplineDrivetrain::SetGoal(
    const ::frc971::control_loops::drivetrain::Goal *goal) {
  if (goal->has_spline_handle()) {
    commanded_spline_ = goal->spline_handle();
  } else {
    commanded_spline_.reset();
  }
  UpdateSplineHandles();
}

bool SplineDrivetrain::HasTrajectory(const fb::Trajectory *trajectory) const {
  if (trajectory == nullptr) {
    return false;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii]->spline_handle() == trajectory->handle()) {
      return true;
    }
  }
  return false;
}

void SplineDrivetrain::AddTrajectory(const fb::Trajectory *trajectory) {
  trajectories_.emplace_back(
      std::make_unique<FinishedTrajectory>(dt_config_, trajectory));
  UpdateSplineHandles();
}

void SplineDrivetrain::DeleteCurrentSpline() {
  CHECK(current_trajectory_index_);
  CHECK_LT(*current_trajectory_index_, trajectories_.size());
  trajectories_.erase(trajectories_.begin() + *current_trajectory_index_);
  executing_spline_ = false;
  current_trajectory_index_.reset();
  current_xva_.setZero();
}

void SplineDrivetrain::UpdateSplineHandles() {
  // If we are currently executing a spline and have received a change
  if (executing_spline_) {
    if (!commanded_spline_) {
      // We've been told to stop executing a spline; remove it from our queue,
      // and clean up.
      DeleteCurrentSpline();
      return;
    } else {
      if (executing_spline_ &&
          current_trajectory().spline_handle() != *commanded_spline_) {
        // If we are executing a spline, and the handle has changed, garbage
        // collect the old spline.
        DeleteCurrentSpline();
      }
    }
  }
  // We've now cleaned up the previous state; handle any new commands.
  if (!commanded_spline_) {
    return;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii]->spline_handle() == *commanded_spline_) {
      executing_spline_ = true;
      current_trajectory_index_ = ii;
    }
  }
  // If we didn't find the commanded spline in the list of available splines,
  // that's fine; it just means, it hasn't been fully planned yet.
}

// TODO(alex): Hold position when done following the spline.
void SplineDrivetrain::Update(
    bool enable, const ::Eigen::Matrix<double, 5, 1> &state,
    const ::Eigen::Matrix<double, 2, 1> &voltage_error) {
  next_U_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  enable_ = enable;
  if (enable && executing_spline_) {
    ::Eigen::Matrix<double, 2, 1> U_ff = ::Eigen::Matrix<double, 2, 1>::Zero();
    if (!IsAtEnd() && executing_spline_) {
      // TODO(alex): It takes about a cycle for the outputs to propagate to the
      // motors. Consider delaying the output by a cycle.
      U_ff = current_trajectory().FFVoltage(current_xva_(0));
    }

    const double current_distance = current_xva_(0);
    ::Eigen::Matrix<double, 2, 5> K =
        current_trajectory().GainForDistance(current_distance);
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    const bool backwards = current_trajectory().drive_spline_backwards();
    if (backwards) {
      ::Eigen::Matrix<double, 2, 1> swapU(U_ff(1, 0), U_ff(0, 0));
      U_ff = -swapU;
      goal_state(2, 0) += M_PI;
      double left_goal = goal_state(3, 0);
      double right_goal = goal_state(4, 0);
      goal_state(3, 0) = -right_goal;
      goal_state(4, 0) = -left_goal;
    }
    const Eigen::Matrix<double, 5, 1> relative_goal =
        current_trajectory().StateToPathRelativeState(current_distance,
                                                      goal_state, backwards);
    const Eigen::Matrix<double, 5, 1> relative_state =
        current_trajectory().StateToPathRelativeState(current_distance, state,
                                                      backwards);
    Eigen::Matrix<double, 5, 1> state_error = relative_goal - relative_state;
    state_error(2, 0) = ::aos::math::NormalizeAngle(state_error(2, 0));
    ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;

    if (backwards) {
      Eigen::Matrix<double, 2, 1> swapU(U_fb(1), U_fb(0));
      U_fb = -swapU;
    }

    ::Eigen::Matrix<double, 2, 1> xv_state = current_xva_.block<2, 1>(0, 0);
    next_xva_ = current_trajectory().GetNextXVA(dt_config_.dt, &xv_state);
    next_U_ = U_ff + U_fb - voltage_error;
    uncapped_U_ = next_U_;
    ScaleCapU(&next_U_);
  }
}

void SplineDrivetrain::SetOutput(
    ::frc971::control_loops::drivetrain::OutputT *output) {
  if (!output) {
    return;
  }
  if (executing_spline_) {
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
  int *spline_handles;
  const flatbuffers::Offset<flatbuffers::Vector<int>> handles_vector =
      builder->CreateUninitializedVector(trajectories_.size(), &spline_handles);

  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    spline_handles[ii] = trajectories_[ii]->spline_handle();
  }

  drivetrain::TrajectoryLogging::Builder trajectory_logging_builder(*builder);
  if (executing_spline_) {
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    trajectory_logging_builder.add_x(goal_state(0));
    trajectory_logging_builder.add_y(goal_state(1));
    if (current_trajectory().drive_spline_backwards()) {
      trajectory_logging_builder.add_left_velocity(-goal_state(4));
      trajectory_logging_builder.add_right_velocity(-goal_state(3));
      trajectory_logging_builder.add_theta(
          ::aos::math::NormalizeAngle(goal_state(2) + M_PI));
    } else {
      trajectory_logging_builder.add_theta(
          ::aos::math::NormalizeAngle(goal_state(2)));
      trajectory_logging_builder.add_left_velocity(goal_state(3));
      trajectory_logging_builder.add_right_velocity(goal_state(4));
    }
  }
  trajectory_logging_builder.add_is_executing(!IsAtEnd() &&
                                              executing_spline_);
  trajectory_logging_builder.add_is_executed(executing_spline_ && IsAtEnd());
  if (commanded_spline_) {
    trajectory_logging_builder.add_goal_spline_handle(*commanded_spline_);
    if (executing_spline_) {
      trajectory_logging_builder.add_current_spline_idx(*commanded_spline_);
    }
  }
  trajectory_logging_builder.add_distance_remaining(
      executing_spline_ ? current_trajectory().length() - current_xva_.x()
                        : 0.0);
  trajectory_logging_builder.add_available_splines(handles_vector);

  return trajectory_logging_builder.Finish();
}

flatbuffers::Offset<TrajectoryLogging> SplineDrivetrain::MakeTrajectoryLogging(
    aos::Sender<drivetrain::Status>::Builder *builder) const {
  return MakeTrajectoryLogging(builder->fbb());
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
