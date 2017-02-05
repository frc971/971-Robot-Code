#ifndef FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
#define FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_

#include <array>
#include <memory>
#include <utility>

#include "Eigen/Dense"

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/simple_capped_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace control_loops {

template <int number_of_states, int number_of_axes>
class ProfiledSubsystem {
 public:
  ProfiledSubsystem(
      ::std::unique_ptr<::frc971::control_loops::SimpleCappedStateFeedbackLoop<
          number_of_states, number_of_axes, number_of_axes>>
          loop,
      ::std::array<::frc971::zeroing::ZeroingEstimator, number_of_axes>
          &&estimators)
      : loop_(::std::move(loop)), estimators_(::std::move(estimators)) {
    zeroed_.fill(false);
    unprofiled_goal_.setZero();
  }

  // Returns whether an error has occured
  bool error() const {
    for (const auto &estimator : estimators_) {
      if (estimator.error()) {
        return true;
      }
    }
    return false;
  }

  void Reset() {
    zeroed_.fill(false);
    for (auto &estimator : estimators_) {
      estimator.Reset();
    }
  }

  // Returns the controller.
  const StateFeedbackLoop<number_of_states, number_of_axes, number_of_axes>
      &controller() const {
    return *loop_;
  }

  int controller_index() const { return loop_->controller_index(); }

  // Returns whether the estimators have been initialized and zeroed.
  bool initialized() const { return initialized_; }

  bool zeroed() const {
    for (int i = 0; i < number_of_axes; ++i) {
      if (!zeroed_[i]) {
        return false;
      }
    }
    return true;
  }

  bool zeroed(int index) const { return zeroed_[index]; };

  // Returns the filtered goal.
  const Eigen::Matrix<double, number_of_states, 1> &goal() const {
    return loop_->R();
  }
  double goal(int row, int col) const { return loop_->R(row, col); }

  // Returns the unprofiled goal.
  const Eigen::Matrix<double, number_of_states, 1> &unprofiled_goal() const {
    return unprofiled_goal_;
  }
  double unprofiled_goal(int row, int col) const {
    return unprofiled_goal_(row, col);
  }

  // Returns the current state estimate.
  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return loop_->X_hat();
  }
  double X_hat(int row, int col) const { return loop_->X_hat(row, col); }

  // Returns the current internal estimator state for logging.
  ::frc971::EstimatorState EstimatorState(int index) {
    ::frc971::EstimatorState estimator_state;
    ::frc971::zeroing::PopulateEstimatorState(estimators_[index],
                                              &estimator_state);

    return estimator_state;
  }

  // Sets the maximum voltage that will be commanded by the loop.
  void set_max_voltage(::std::array<double, number_of_axes> voltages) {
    for (int i = 0; i < number_of_axes; ++i) {
      loop_->set_max_voltage(i, voltages[i]);
    }
  }

 protected:
  void set_zeroed(int index, bool val) { zeroed_[index] = val; }

  // TODO(austin): It's a bold assumption to assume that we will have the same
  // number of sensors as axes.  So far, that's been fine.
  ::std::unique_ptr<::frc971::control_loops::SimpleCappedStateFeedbackLoop<
      number_of_states, number_of_axes, number_of_axes>>
      loop_;

  // The goal that the profile tries to reach.
  Eigen::Matrix<double, number_of_states, 1> unprofiled_goal_;

  bool initialized_ = false;

  ::std::array<::frc971::zeroing::ZeroingEstimator, number_of_axes> estimators_;

 private:
  ::std::array<bool, number_of_axes> zeroed_;
};

class SingleDOFProfiledSubsystem
    : public ::frc971::control_loops::ProfiledSubsystem<3, 1> {
 public:
  SingleDOFProfiledSubsystem(
      ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
      const ::frc971::constants::ZeroingConstants &estimator,
      const ::frc971::constants::Range &range, double default_angular_velocity,
      double default_angular_acceleration);

  // Updates our estimator with the latest position.
  void Correct(::frc971::PotAndIndexPosition position);
  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);

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

  // Returns the requested intake voltage.
  double intake_voltage() const { return loop_->U(0, 0); }

  // Returns the current position.
  double angle() const { return Y_(0, 0); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { estimators_[0].TriggerError(); }

 private:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  void CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal);

  void UpdateOffset(double offset);

  aos::util::TrapezoidProfile profile_;

  // Current measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 1, 1> offset_;

  const ::frc971::constants::Range range_;

  const double default_velocity_;
  const double default_acceleration_;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
