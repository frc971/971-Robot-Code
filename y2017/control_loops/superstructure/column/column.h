#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_COLUMN_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_COLUMN_H_

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include "Eigen/Dense"

#include "frc971/constants.h"
#include "frc971/control_loops/profiled_subsystem.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2017/constants.h"
#include "y2017/control_loops/superstructure/column/column_zeroing.h"
#include "y2017/control_loops/superstructure/intake/intake.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"
#include "y2017/control_loops/superstructure/vision_time_adjuster.h"
#include "y2017/vision/vision.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace column {

class ColumnProfiledSubsystem
    : public ::frc971::control_loops::ProfiledSubsystem<
          6, 1, ColumnZeroingEstimator, 2, 2> {
 public:
  ColumnProfiledSubsystem(
      ::std::unique_ptr<
          ::frc971::control_loops::SimpleCappedStateFeedbackLoop<6, 2, 2>>
          loop,
      const ::y2017::constants::Values::Column &zeroing_constants,
      const ::frc971::constants::Range &range, double default_angular_velocity,
      double default_angular_acceleration);

  // Updates our estimator with the latest position.
  void Correct(const ColumnPosition &position);
  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);

  // Fills out the ProfiledJointStatus structure with the current state.
  template <class StatusType>
  void PopulateTurretStatus(StatusType *status);

  // Forces the current goal to the provided goal, bypassing the profiler.
  void ForceGoal(double goal_velocity, double goal);
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_indexer_unprofiled_goal(double goal_velocity);
  void set_turret_unprofiled_goal(double unprofiled_goal);
  void set_unprofiled_goal(double goal_velocity, double unprofiled_goal);
  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(const ::frc971::ProfileParameters &profile_parameters);
  void AdjustProfile(double max_angular_velocity,
                     double max_angular_acceleration);

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();

  // Returns the requested voltage.
  double indexer_voltage() const { return loop_->U(0, 0); }
  double uncapped_indexer_voltage() const { return loop_->U_uncapped(0, 0); }
  double turret_voltage() const { return loop_->U(1, 0); }

  // Returns the current Y.
  const ::Eigen::Matrix<double, 2, 1> Y() const { return Y_; }
  double Y(int i, int j) const { return Y_(i, j); }

  // Returns the current indexer position.
  double indexer_position() const { return Y_(0, 0); }

  bool saturated() const { return saturated_; }

  // Returns the current turret position.
  double turret_position() const { return Y_(1, 0) + Y_(0, 0); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { estimators_[0].TriggerError(); }

  const ::frc971::constants::Range &range() const { return range_; }

  bool IsIndexerStuck() const;
  double IndexerStuckVoltage() const;
  void PartialIndexerReset();
  void PartialTurretReset();
  void PopulateIndexerStatus(IndexerStatus *status);

  void AddOffset(double indexer_offset_delta, double turret_offset_delta);

 protected:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  virtual void CapGoal(const char *name, Eigen::Matrix<double, 6, 1> *goal);

 private:
  void UpdateOffset(double indexer_offset, double turret_offset);

  ::std::unique_ptr<StateFeedbackLoop<6, 2, 2>> stuck_indexer_detector_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 5;
  ::std::array<double, kHistoryLength> indexer_history_;
  ptrdiff_t indexer_history_position_ = 0;

  double indexer_error_ = 0.0;
  double indexer_dt_velocity_ = 0.0;
  double indexer_last_position_ = 0.0;
  double indexer_average_angular_velocity_ = 0.0;
  double indexer_position_error_ = 0.0;
  bool indexer_ready_ = false;

  bool saturated_ = false;

  Eigen::Matrix<double, 6, 1> X_hat_current_;
  Eigen::Matrix<double, 6, 1> stuck_indexer_X_hat_current_;

  aos::util::TrapezoidProfile profile_;

  // Current measurement.
  Eigen::Matrix<double, 2, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 2, 1> offset_;

  const ::frc971::constants::Range range_;

  const double default_velocity_;
  const double default_acceleration_;

  double turret_last_position_ = 0;
};

template <typename StatusType>
void ColumnProfiledSubsystem::PopulateTurretStatus(StatusType *status) {
  status->zeroed = zeroed();
  status->state = -1;
  // We don't know, so default to the bad case.
  status->estopped = true;

  status->position = X_hat(2, 0);
  status->velocity = X_hat(3, 0);
  status->goal_position = goal(2, 0);
  status->goal_velocity = goal(3, 0);
  status->unprofiled_goal_position = unprofiled_goal(2, 0);
  status->unprofiled_goal_velocity = unprofiled_goal(3, 0);
  status->voltage_error = X_hat(5, 0);
  status->calculated_velocity =
      (turret_position() - turret_last_position_) /
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(
          ::aos::controls::kLoopFrequency)
          .count();

  status->estimator_state = EstimatorState(0);

  Eigen::Matrix<double, 6, 1> error = controller().error();
  status->position_power = controller().controller().K(0, 0) * error(0, 0);
  status->velocity_power = controller().controller().K(0, 1) * error(1, 0);
}

class Column {
 public:
  Column();
  double goal(int row, int col) const {
    return profiled_subsystem_.goal(row, col);
  }

  double turret_position() const {
    return profiled_subsystem_.turret_position();
  }

  void set_freeze(bool freeze) { freeze_ = freeze; }

  // The zeroing and operating voltages.
  static constexpr double kZeroingVoltage = 5.0;
  static constexpr double kOperatingVoltage = 12.0;
  static constexpr double kIntakeZeroingMinDistance = 0.08;
  static constexpr double kIntakeTolerance = 0.005;
  static constexpr double kStuckZeroingTrackingError = 0.02;
  static constexpr double kTurretMin = -0.1;
  static constexpr double kTurretMax = M_PI / 2.0 + 0.1;

  void Iterate(const control_loops::IndexerGoal *unsafe_indexer_goal,
               const control_loops::TurretGoal *unsafe_turret_goal,
               const ColumnPosition *position,
               const vision::VisionStatus *vision_status,
               double *indexer_output, double *turret_output,
               IndexerStatus *indexer_status,
               TurretProfiledSubsystemStatus *turret_status,
               intake::Intake *intake);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED = 0,
    ZEROING_UNINITIALIZED = 1,
    ZEROING_POSITIVE = 2,
    ZEROING_NEGATIVE = 3,
    RUNNING = 4,
    ESTOP = 5,
  };

  enum class IndexerState : int32_t {
    // The system is running correctly, no stuck condition detected.
    RUNNING = 0,
    // The system is reversing to unjam.
    REVERSING = 1
  };

  State state() const { return state_; }
  IndexerState indexer_state() const { return indexer_state_; }

 private:
  State state_ = State::UNINITIALIZED;

  IndexerState indexer_state_ = IndexerState::RUNNING;

  bool freeze_ = false;

  VisionTimeAdjuster vision_time_adjuster_;

  // The last time that we transitioned from RUNNING to REVERSING or the
  // reverse.  Used to implement the timeouts.
  ::aos::monotonic_clock::time_point last_transition_time_ =
      ::aos::monotonic_clock::min_time;

  ColumnProfiledSubsystem profiled_subsystem_;

  const double vision_error_;
};

}  // namespace column
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_COLUMN_COLUMN_H_
