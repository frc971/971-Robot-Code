#ifndef Y2017_CONTROL_LOOPS_INDEXER_INDEXER_H_
#define Y2017_CONTROL_LOOPS_INDEXER_INDEXER_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2017/control_loops/superstructure/indexer/indexer_integral_plant.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace indexer {

class IndexerController {
 public:
  IndexerController();

  // Sets the velocity goal in radians/sec
  void set_goal(double angular_velocity_goal);
  // Sets the current encoder position in radians
  void set_position(double current_position);

  // Populates the status structure.
  void SetStatus(control_loops::IndexerStatus *status);

  // Returns the control loop calculated voltage.
  double voltage() const;

  // Returns the instantaneous velocity.
  double velocity() const { return loop_->X_hat(1, 0); }

  double dt_velocity() const { return dt_velocity_; }

  double error() const { return error_; }

  // Returns true if the indexer is stuck.
  bool IsStuck() const;
  double StuckVoltage() const;

  // Executes the control loop for a cycle.
  void Update(bool disabled);

  // Resets the kalman filter and any other internal state.
  void Reset();
  // Resets the voltage error for when we reverse directions.
  void PartialReset();

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // The control loop.
  ::std::unique_ptr<StateFeedbackLoop<3, 1, 1>> loop_;

  // The stuck indexer detector
  ::std::unique_ptr<StateFeedbackLoop<3, 1, 1>> stuck_indexer_detector_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 5;
  ::std::array<double, kHistoryLength> history_;
  ptrdiff_t history_position_ = 0;

  double error_ = 0.0;
  double dt_velocity_ = 0.0;
  double last_position_ = 0.0;
  double average_angular_velocity_ = 0.0;
  double position_error_ = 0.0;

  Eigen::Matrix<double, 3, 1> X_hat_current_;
  Eigen::Matrix<double, 3, 1> stuck_indexer_X_hat_current_;

  bool ready_ = false;
  bool reset_ = false;

  DISALLOW_COPY_AND_ASSIGN(IndexerController);
};

class Indexer {
 public:
  Indexer() {}

  enum class State {
    // The system is running correctly, no stuck condition detected.
    RUNNING = 0,
    // The system is reversing to unjam.
    REVERSING = 1
  };

  // Iterates the indexer control loop one cycle.  position and status must
  // never be nullptr.  goal can be nullptr if no goal exists, and output should
  // be nullptr if disabled.
  void Iterate(const control_loops::IndexerGoal *goal, const double *position,
               double *output, control_loops::IndexerStatus *status);

  // Sets the indexer up to reset the kalman filter next time Iterate is called.
  void Reset();

  State state() const { return state_; }

 private:
  IndexerController indexer_;

  State state_ = State::RUNNING;

  // The last time that we transitioned from RUNNING to REVERSING or the
  // reverse.  Used to implement the timeouts.
  ::aos::monotonic_clock::time_point last_transition_time_ =
      ::aos::monotonic_clock::min_time;

  DISALLOW_COPY_AND_ASSIGN(Indexer);
};

}  // namespace indexer
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_INDEXER_INDEXER_H_
