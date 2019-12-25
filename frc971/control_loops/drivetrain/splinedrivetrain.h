#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_

#include <atomic>
#include <thread>

#include "Eigen/Dense"

#include "aos/condition.h"
#include "aos/mutex/mutex.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/distance_spline.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "frc971/control_loops/drivetrain/trajectory.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class SplineDrivetrain {
 public:
  SplineDrivetrain(const DrivetrainConfig<double> &dt_config);

  ~SplineDrivetrain() {
    {
      ::aos::MutexLocker locker(&mutex_);
      run_ = false;
      new_goal_.Signal();
    }
    worker_thread_.join();
  }

  void SetGoal(const ::frc971::control_loops::drivetrain::Goal *goal);

  void Update(bool enabled, const ::Eigen::Matrix<double, 5, 1> &state);

  void SetOutput(::frc971::control_loops::drivetrain::OutputT *output);

  flatbuffers::Offset<TrajectoryLogging> MakeTrajectoryLogging(
      aos::Sender<drivetrain::Status>::Builder *builder) const;
  flatbuffers::Offset<TrajectoryLogging> MakeTrajectoryLogging(
      flatbuffers::FlatBufferBuilder *builder) const;
  void PopulateStatus(
      drivetrain::Status::Builder *status) const;

  // Accessor for the current goal state, pretty much only present for debugging
  // purposes.
  ::Eigen::Matrix<double, 5, 1> CurrentGoalState() const {
    return current_trajectory_
               ? current_trajectory_->GoalState(current_xva_(0),
                                                current_xva_(1))
               : ::Eigen::Matrix<double, 5, 1>::Zero();
  }

  bool IsAtEnd() const {
    return current_trajectory_
        ? current_trajectory_->is_at_end(current_xva_.block<2, 1>(0, 0)) :
              true;
  }

  // Returns true if the splinedrivetrain is enabled.
  bool enable() const { return enable_; }

  enum class PlanState : int8_t {
    kNoPlan = 0,
    kBuildingTrajectory = 1,
    kPlanningTrajectory = 2,
    kPlannedTrajectory = 3,
  };

 private:
  void ComputeTrajectory();
  void ScaleCapU(Eigen::Matrix<double, 2, 1> *U);

  const DrivetrainConfig<double> dt_config_;

  int32_t current_spline_handle_ = 0;  // Current spline told to excecute.
  int32_t current_spline_idx_ = 0;     // Current executing spline.
  bool has_started_execution_ = false;

  ::std::unique_ptr<DistanceSpline> current_distance_spline_;
  ::std::unique_ptr<Trajectory> current_trajectory_;
  bool current_drive_spline_backwards_ = false;

  // State required to compute the next iteration's output.
  ::Eigen::Matrix<double, 3, 1> current_xva_, next_xva_;
  ::Eigen::Matrix<double, 2, 1> next_U_;

  // Information used for status message.
  ::Eigen::Matrix<double, 2, 1> uncapped_U_;
  bool enable_ = false;
  bool output_was_capped_ = false;

  std::atomic<PlanningState> plan_state_ = {PlanningState::NO_PLAN};

  ::std::thread worker_thread_;
  // mutex_ is held by the worker thread while it is doing work or by the main
  // thread when it is sending work to the worker thread.
  ::aos::Mutex mutex_;
  // new_goal_ is used to signal to the worker thread that ther is work to do.
  ::aos::Condition new_goal_;
  // The following variables are guarded by mutex_.
  bool run_ = true;

  // These two structures mirror the flatbuffer Multispline.
  // TODO(austin): copy the goal flatbuffer directly instead of recreating it
  // like this...
  struct MultiSpline {
    int32_t spline_count;
    std::array<float, 36> spline_x;
    std::array<float, 36> spline_y;
    std::array<ConstraintT, 6> constraints;
  };

  struct SplineGoal {
    int32_t spline_idx = 0;

    bool drive_spline_backwards;

    MultiSpline spline;
  };

  SplineGoal goal_;
  ::std::unique_ptr<DistanceSpline> past_distance_spline_;
  ::std::unique_ptr<DistanceSpline> future_distance_spline_;
  ::std::unique_ptr<Trajectory> past_trajectory_;
  ::std::unique_ptr<Trajectory> future_trajectory_;
  bool future_drive_spline_backwards_ = false;
  int32_t future_spline_idx_ = 0;  // Current spline being computed.
  ::std::atomic<int32_t> planning_spline_idx_{-1};

  // TODO(alex): pull this out of dt_config.
  const ::Eigen::DiagonalMatrix<double, 5> Q =
      (::Eigen::DiagonalMatrix<double, 5>().diagonal()
           << 1.0 / ::std::pow(0.12, 2),
       1.0 / ::std::pow(0.12, 2), 1.0 / ::std::pow(0.1, 2),
       1.0 / ::std::pow(1.5, 2), 1.0 / ::std::pow(1.5, 2))
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
