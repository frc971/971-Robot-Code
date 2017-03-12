#ifndef FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
#define FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include "Eigen/Dense"

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/control_loops/profiled_subsystem.q.h"
#include "frc971/control_loops/simple_capped_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/zeroing/zeroing.h"
#include "frc971/constants.h"

namespace frc971 {
namespace control_loops {

// TODO(Brian): Use a tuple instead of an array to support heterogeneous zeroing
// styles.
template <int number_of_states, int number_of_axes,
          class ZeroingEstimator =
              ::frc971::zeroing::PotAndIndexPulseZeroingEstimator,
          int number_of_inputs = number_of_axes,
          int number_of_outputs = number_of_axes>
class ProfiledSubsystem {
 public:
  ProfiledSubsystem(
      ::std::unique_ptr<::frc971::control_loops::SimpleCappedStateFeedbackLoop<
          number_of_states, number_of_inputs, number_of_outputs>>
          loop,
      ::std::array<ZeroingEstimator, number_of_axes> &&estimators)
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
    initialized_ = false;
    for (auto &estimator : estimators_) {
      estimator.Reset();
    }
    should_reset_ = true;
  }

  // Returns the controller.
  const StateFeedbackLoop<number_of_states, number_of_inputs, number_of_outputs> &
  controller() const {
    return *loop_;
  }

  int controller_index() const { return loop_->index(); }

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
  double &mutable_X_hat(int row, int col) const {
    return loop_->mutable_X_hat(row, col);
  }

  // Returns the current internal estimator state for logging.
  typename ZeroingEstimator::State EstimatorState(int index) {
    return estimators_[index].GetEstimatorState();
  }

  // Sets the maximum voltage that will be commanded by the loop.
  void set_max_voltage(::std::array<double, number_of_inputs> voltages) {
    for (int i = 0; i < number_of_inputs; ++i) {
      loop_->set_max_voltage(i, voltages[i]);
    }
  }

 protected:
  void set_zeroed(int index, bool val) { zeroed_[index] = val; }

  ::std::unique_ptr<::frc971::control_loops::SimpleCappedStateFeedbackLoop<
      number_of_states, number_of_inputs, number_of_outputs>>
      loop_;

  // The goal that the profile tries to reach.
  Eigen::Matrix<double, number_of_states, 1> unprofiled_goal_;

  bool initialized_ = false;

  // If true, the subclass should reset in Update.  It should then clear this
  // flag.
  bool should_reset_ = true;

  ::std::array<ZeroingEstimator, number_of_axes> estimators_;

 private:
  ::std::array<bool, number_of_axes> zeroed_;
};

template <typename ZeroingEstimator =
              ::frc971::zeroing::PotAndIndexPulseZeroingEstimator>
class SingleDOFProfiledSubsystem
    : public ::frc971::control_loops::ProfiledSubsystem<3, 1, ZeroingEstimator> {
 public:
  SingleDOFProfiledSubsystem(
      ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
      const typename ZeroingEstimator::ZeroingConstants &zeroing_constants,
      const ::frc971::constants::Range &range, double default_angular_velocity,
      double default_angular_acceleration);

  // Updates our estimator with the latest position.
  void Correct(typename ZeroingEstimator::Position position);
  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);

  // Fills out the ProfiledJointStatus structure with the current state.
  template <class StatusType>
  void PopulateStatus(StatusType *status);

  // Forces the current goal to the provided goal, bypassing the profiler.
  void ForceGoal(double goal);
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_unprofiled_goal(double unprofiled_goal);
  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(const ::frc971::ProfileParameters &profile_parameters);
  void AdjustProfile(double max_angular_velocity,
                     double max_angular_acceleration);

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();

  // Returns the requested voltage.
  double voltage() const { return this->loop_->U(0, 0); }

  // Returns the current position.
  double position() const { return this->Y_(0, 0); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { this->estimators_[0].TriggerError(); }

  const ::frc971::constants::Range &range() const { return range_; }

 protected:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  virtual void CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal);

 private:
  void UpdateOffset(double offset);

  aos::util::TrapezoidProfile profile_;

  // Current measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 1, 1> offset_;

  const ::frc971::constants::Range range_;

  const double default_velocity_;
  const double default_acceleration_;

  double last_position_ = 0;
};

namespace internal {

double UseUnlessZero(double target_value, double default_value);

}  // namespace internal

template <class ZeroingEstimator>
SingleDOFProfiledSubsystem<ZeroingEstimator>::SingleDOFProfiledSubsystem(
    ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
    const typename ZeroingEstimator::ZeroingConstants &zeroing_constants,
    const ::frc971::constants::Range &range, double default_velocity,
    double default_acceleration)
    : ProfiledSubsystem<3, 1, ZeroingEstimator>(::std::move(loop),
                                                {{zeroing_constants}}),
      profile_(::aos::controls::kLoopFrequency),
      range_(range),
      default_velocity_(default_velocity),
      default_acceleration_(default_acceleration) {
  Y_.setZero();
  offset_.setZero();
  AdjustProfile(0.0, 0.0);
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::UpdateOffset(double offset) {
  const double doffset = offset - offset_(0, 0);
  LOG(INFO, "Adjusting offset from %f to %f\n", offset_(0, 0), offset);

  this->loop_->mutable_X_hat()(0, 0) += doffset;
  this->Y_(0, 0) += doffset;
  last_position_ += doffset;
  this->loop_->mutable_R(0, 0) += doffset;

  profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &this->loop_->mutable_R());
}

template <class ZeroingEstimator>
template <class StatusType>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::PopulateStatus(
    StatusType *status) {
  status->zeroed = this->zeroed();
  status->state = -1;
  // We don't know, so default to the bad case.
  status->estopped = true;

  status->position = this->X_hat(0, 0);
  status->velocity = this->X_hat(1, 0);
  status->goal_position = this->goal(0, 0);
  status->goal_velocity = this->goal(1, 0);
  status->unprofiled_goal_position = this->unprofiled_goal(0, 0);
  status->unprofiled_goal_velocity = this->unprofiled_goal(1, 0);
  status->voltage_error = this->X_hat(2, 0);
  status->calculated_velocity =
      (position() - last_position_) /
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(
          ::aos::controls::kLoopFrequency)
          .count();

  status->estimator_state = this->EstimatorState(0);

  Eigen::Matrix<double, 3, 1> error = this->controller().error();
  status->position_power =
      this->controller().controller().K(0, 0) * error(0, 0);
  status->velocity_power =
      this->controller().controller().K(0, 1) * error(1, 0);
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::Correct(
    typename ZeroingEstimator::Position new_position) {
  this->estimators_[0].UpdateEstimate(new_position);

  if (this->estimators_[0].error()) {
    LOG(ERROR, "zeroing error\n");
    return;
  }

  if (!this->initialized_) {
    if (this->estimators_[0].offset_ready()) {
      UpdateOffset(this->estimators_[0].offset());
      this->initialized_ = true;
    }
  }

  if (!this->zeroed(0) && this->estimators_[0].zeroed()) {
    UpdateOffset(this->estimators_[0].offset());
    this->set_zeroed(0, true);
  }

  last_position_ = position();
  this->Y_ << new_position.encoder;
  this->Y_ += this->offset_;
  this->loop_->Correct(Y_);
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::CapGoal(
    const char *name, Eigen::Matrix<double, 3, 1> *goal) {
  // Limit the goal to min/max allowable positions.
  if ((*goal)(0, 0) > range_.upper) {
    LOG(WARNING, "Goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
        range_.upper);
    (*goal)(0, 0) = range_.upper;
  }
  if ((*goal)(0, 0) < range_.lower) {
    LOG(WARNING, "Goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
        range_.lower);
    (*goal)(0, 0) = range_.lower;
  }
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::ForceGoal(double goal) {
  set_unprofiled_goal(goal);
  this->loop_->mutable_R() = this->unprofiled_goal_;
  this->loop_->mutable_next_R() = this->loop_->R();

  const ::Eigen::Matrix<double, 3, 1> &R = this->loop_->R();
  this->profile_.MoveCurrentState(R.block<2, 1>(0, 0));
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::set_unprofiled_goal(
    double unprofiled_goal) {
  this->unprofiled_goal_(0, 0) = unprofiled_goal;
  this->unprofiled_goal_(1, 0) = 0.0;
  this->unprofiled_goal_(2, 0) = 0.0;
  CapGoal("unprofiled R", &this->unprofiled_goal_);
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::Update(bool disable) {
  // TODO(austin): What do we want to do with the profile on reset?  Also, we
  // should probably reset R, the offset, the profile, etc.
  if (this->should_reset_) {
    this->loop_->mutable_X_hat(0, 0) = Y_(0, 0);
    this->loop_->mutable_X_hat(1, 0) = 0.0;
    this->loop_->mutable_X_hat(2, 0) = 0.0;
    this->should_reset_ = false;
  }

  if (!disable) {
    ::Eigen::Matrix<double, 2, 1> goal_state = profile_.Update(
        this->unprofiled_goal_(0, 0), this->unprofiled_goal_(1, 0));

    this->loop_->mutable_next_R(0, 0) = goal_state(0, 0);
    this->loop_->mutable_next_R(1, 0) = goal_state(1, 0);
    this->loop_->mutable_next_R(2, 0) = 0.0;
    CapGoal("next R", &this->loop_->mutable_next_R());
  }

  this->loop_->Update(disable);

  if (!disable && this->loop_->U(0, 0) != this->loop_->U_uncapped(0, 0)) {
    const ::Eigen::Matrix<double, 3, 1> &R = this->loop_->R();
    profile_.MoveCurrentState(R.block<2, 1>(0, 0));
  }
}

template <class ZeroingEstimator>
bool SingleDOFProfiledSubsystem<ZeroingEstimator>::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (position() > range_.upper_hard || position() < range_.lower_hard) {
    LOG(ERROR,
        "SingleDOFProfiledSubsystem at %f out of bounds [%f, %f], ESTOPing\n",
        position(), range_.lower_hard, range_.upper_hard);
    return true;
  }

  return false;
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::AdjustProfile(
    const ::frc971::ProfileParameters &profile_parameters) {
  AdjustProfile(profile_parameters.max_velocity,
                profile_parameters.max_acceleration);
}

template <class ZeroingEstimator>
void SingleDOFProfiledSubsystem<ZeroingEstimator>::AdjustProfile(
    double max_angular_velocity, double max_angular_acceleration) {
  profile_.set_maximum_velocity(
      internal::UseUnlessZero(max_angular_velocity, default_velocity_));
  profile_.set_maximum_acceleration(
      internal::UseUnlessZero(max_angular_acceleration, default_acceleration_));
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
