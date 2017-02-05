#ifndef Y2016_BOT3_CONTROL_LOOPS_INTAKE_INTAKE_CONTROLS_H_
#define Y2016_BOT3_CONTROL_LOOPS_INTAKE_INTAKE_CONTROLS_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/simple_capped_state_feedback_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"

namespace y2016_bot3 {
namespace control_loops {
namespace intake {
namespace testing {
class IntakeTest_DisabledGoalTest_Test;
}  // namespace testing

class IntakeArm {
 public:
  IntakeArm();
  // Returns whether the estimators have been initialized and zeroed.
  bool initialized() const { return initialized_; }
  bool zeroed() const { return zeroed_; }
  // Returns whether an error has occured
  bool error() const { return estimator_.error(); }

  // Updates our estimator with the latest position.
  void Correct(::frc971::PotAndIndexPosition position);
  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);
  // Sets the maximum voltage that will be commanded by the loop.
  void set_max_voltage(double voltage);

  // Forces the current goal to the provided goal, bypassing the profiler.
  void ForceGoal(double goal);
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_unprofiled_goal(double unprofiled_goal);
  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(double max_angular_velocity,
                     double max_angular_acceleration);

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();
  // Resets the internal state.
  void Reset();

  // Returns the current internal estimator state for logging.
  ::frc971::EstimatorState IntakeEstimatorState();

  // Returns the requested intake voltage.
  double intake_voltage() const { return loop_->U(0, 0); }

  // Returns the current position.
  double angle() const { return Y_(0, 0); }

  // Returns the controller error.
  const StateFeedbackLoop<3, 1, 1> &controller() const { return *loop_; }

  // Returns the filtered goal.
  const Eigen::Matrix<double, 3, 1> &goal() const { return loop_->R(); }
  double goal(int row, int col) const { return loop_->R(row, col); }

  // Returns the unprofiled goal.
  const Eigen::Matrix<double, 3, 1> &unprofiled_goal() const {
    return unprofiled_goal_;
  }
  double unprofiled_goal(int row, int col) const {
    return unprofiled_goal_(row, col);
  }

  // Returns the current state estimate.
  const Eigen::Matrix<double, 3, 1> &X_hat() const { return loop_->X_hat(); }
  double X_hat(int row, int col) const { return loop_->X_hat(row, col); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { estimator_.TriggerError(); }

 private:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  void CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal);

  void UpdateIntakeOffset(double offset);

  ::std::unique_ptr<
      ::frc971::control_loops::SimpleCappedStateFeedbackLoop<3, 1, 1>> loop_;

  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator estimator_;
  aos::util::TrapezoidProfile profile_;

  // Current measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 1, 1> offset_;

  // The goal that the profile tries to reach.
  Eigen::Matrix<double, 3, 1> unprofiled_goal_;

  bool initialized_ = false;
  bool zeroed_ = false;
};

}  // namespace intake
}  // namespace control_loops
}  // namespace y2016_bot3

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_
