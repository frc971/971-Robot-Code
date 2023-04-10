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
      velocity_drivetrain_(
          std::make_shared<StateFeedbackLoop<2, 2, 2, double,
                                             StateFeedbackHybridPlant<2, 2, 2>,
                                             HybridKalman<2, 2, 2>>>(
              dt_config_.make_hybrid_drivetrain_velocity_loop())),
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
  UpdateSplineHandles(goal->has_spline_handle()
                          ? std::make_optional<int>(goal->spline_handle())
                          : std::nullopt);
}

bool SplineDrivetrain::IsCurrentTrajectory(
    const fb::Trajectory *trajectory) const {
  const FinishedTrajectory *current = current_trajectory();
  return (current != nullptr &&
          current->spline_handle() == trajectory->handle());
}

bool SplineDrivetrain::HasTrajectory(const fb::Trajectory *trajectory) const {
  if (trajectory == nullptr) {
    return false;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == trajectory->handle()) {
      return true;
    }
  }
  return false;
}

void SplineDrivetrain::DeleteTrajectory(const fb::Trajectory *trajectory) {
  CHECK(trajectory != nullptr);

  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == trajectory->handle()) {
      trajectories_.erase(trajectories_.begin() + ii);
      return;
    }
  }

  LOG(FATAL) << "Trying to remove unknown trajectory " << trajectory->handle();
}

void SplineDrivetrain::AddTrajectory(const fb::Trajectory *trajectory) {
  CHECK_LT(trajectories_.size(), trajectories_.capacity());
  trajectories_.emplace_back(dt_config_, trajectory, velocity_drivetrain_);
  UpdateSplineHandles(commanded_spline_);
}

void SplineDrivetrain::DeleteCurrentSpline() {
  DeleteTrajectory(&CHECK_NOTNULL(current_trajectory())->trajectory());
  executing_spline_ = false;
  commanded_spline_.reset();
  current_xva_.setZero();
}

void SplineDrivetrain::UpdateSplineHandles(
    std::optional<int> commanded_spline) {
  // If we are currently executing a spline and have received a change
  if (executing_spline_) {
    if (!commanded_spline) {
      // We've been told to stop executing a spline; remove it from our queue,
      // and clean up.
      DeleteCurrentSpline();
      return;
    } else {
      if (executing_spline_ &&
          CHECK_NOTNULL(current_trajectory())->spline_handle() !=
              *commanded_spline) {
        // If we are executing a spline, and the handle has changed, garbage
        // collect the old spline.
        DeleteCurrentSpline();
      }
    }
  }
  commanded_spline_ = commanded_spline;
  // We've now cleaned up the previous state; handle any new commands.
  if (!commanded_spline_) {
    return;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      executing_spline_ = true;
    }
  }
  // If we didn't find the commanded spline in the list of available splines,
  // that's fine; it just means, it hasn't been fully planned yet.
}

FinishedTrajectory *SplineDrivetrain::current_trajectory() {
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      return &trajectories_[ii];
    }
  }
  return nullptr;
}

const FinishedTrajectory *SplineDrivetrain::current_trajectory() const {
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      return &trajectories_[ii];
    }
  }
  return nullptr;
}

// TODO(alex): Hold position when done following the spline.
void SplineDrivetrain::Update(
    bool enable, const ::Eigen::Matrix<double, 5, 1> &state,
    const ::Eigen::Matrix<double, 2, 1> &voltage_error) {
  next_U_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  enable_ = enable;
  if (enable && executing_spline_) {
    ::Eigen::Matrix<double, 2, 1> U_ff = ::Eigen::Matrix<double, 2, 1>::Zero();
    const FinishedTrajectory *const trajectory =
        CHECK_NOTNULL(current_trajectory());
    if (!IsAtEnd() && executing_spline_) {
      // TODO(alex): It takes about a cycle for the outputs to propagate to the
      // motors. Consider delaying the output by a cycle.
      U_ff = trajectory->FFVoltage(current_xva_(0));
    }

    const double current_distance = current_xva_(0);
    ::Eigen::Matrix<double, 2, 5> K =
        trajectory->GainForDistance(current_distance);
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    const bool backwards = trajectory->drive_spline_backwards();
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
        trajectory->StateToPathRelativeState(current_distance, goal_state,
                                             backwards);
    const Eigen::Matrix<double, 5, 1> relative_state =
        trajectory->StateToPathRelativeState(current_distance, state,
                                             backwards);
    Eigen::Matrix<double, 5, 1> state_error = relative_goal - relative_state;
    state_error(2, 0) = ::aos::math::NormalizeAngle(state_error(2, 0));
    ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;

    if (backwards) {
      Eigen::Matrix<double, 2, 1> swapU(U_fb(1), U_fb(0));
      U_fb = -swapU;
    }

    ::Eigen::Matrix<double, 2, 1> xv_state = current_xva_.block<2, 1>(0, 0);
    next_xva_ = trajectory->GetNextXVA(dt_config_.dt, &xv_state);
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
    spline_handles[ii] = trajectories_[ii].spline_handle();
  }

  drivetrain::TrajectoryLogging::Builder trajectory_logging_builder(*builder);
  if (executing_spline_) {
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    trajectory_logging_builder.add_x(goal_state(0));
    trajectory_logging_builder.add_y(goal_state(1));
    if (CHECK_NOTNULL(current_trajectory())->drive_spline_backwards()) {
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
  trajectory_logging_builder.add_is_executing(!IsAtEnd() && executing_spline_);
  trajectory_logging_builder.add_is_executed(executing_spline_ && IsAtEnd());
  if (commanded_spline_) {
    trajectory_logging_builder.add_goal_spline_handle(*commanded_spline_);
    if (executing_spline_) {
      trajectory_logging_builder.add_current_spline_idx(*commanded_spline_);
    }
  }
  trajectory_logging_builder.add_distance_remaining(
      executing_spline_
          ? CHECK_NOTNULL(current_trajectory())->length() - current_xva_.x()
          : 0.0);
  trajectory_logging_builder.add_available_splines(handles_vector);
  trajectory_logging_builder.add_distance_traveled(
      executing_spline_ ? current_xva_.x() : 0.0);

  return trajectory_logging_builder.Finish();
}

flatbuffers::Offset<TrajectoryLogging> SplineDrivetrain::MakeTrajectoryLogging(
    aos::Sender<drivetrain::Status>::Builder *builder) const {
  return MakeTrajectoryLogging(builder->fbb());
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
