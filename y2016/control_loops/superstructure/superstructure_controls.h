#ifndef Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_
#define Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/simple_capped_state_feedback_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {
namespace testing {
class SuperstructureTest_DisabledGoalTest_Test;
}  // namespace testing

class Intake {
 public:
  Intake();
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

  ::frc971::zeroing::ZeroingEstimator estimator_;
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

class ArmControlLoop
    : public ::frc971::control_loops::SimpleCappedStateFeedbackLoop<6, 2, 2> {
 public:
  ArmControlLoop(SimpleCappedStateFeedbackLoop<6, 2, 2> &&loop)
      : SimpleCappedStateFeedbackLoop<6, 2, 2>(::std::move(loop)) {}

  const Eigen::Matrix<double, 2, 1> ControllerOutput() override {
    const Eigen::Matrix<double, 2, 1> accelerating_ff =
        controller(0).Kff * (next_R() - controller(0).plant.A() * R());
    const Eigen::Matrix<double, 2, 1> accelerating_controller =
        controller(0).K * error() + accelerating_ff;

    const Eigen::Matrix<double, 2, 1> decelerating_ff =
        controller(1).Kff * (next_R() - controller(1).plant.A() * R());
    const Eigen::Matrix<double, 2, 1> decelerating_controller =
        controller(1).K * error() + decelerating_ff;

    const double bemf_voltage = X_hat(1, 0) / kV_shoulder;
    bool use_accelerating_controller = true;
    LOG(DEBUG, "Accelerating at %f, decel %f, bemf %f\n",
        accelerating_controller(0, 0), accelerating_controller(1, 0),
        bemf_voltage);
    if (IsAccelerating(bemf_voltage, accelerating_controller(0, 0))) {
      use_accelerating_controller = true;
    } else {
      use_accelerating_controller = false;
    }
    if (use_accelerating_controller) {
      ff_U_ = accelerating_ff;
      set_controller_index(0);
      return accelerating_controller;
    } else {
      set_controller_index(1);
      ff_U_ = decelerating_ff;
      return decelerating_controller;
    }
  }

 private:
  void CapU() override {
    // U(0)
    // U(1) = coupling * U(0) + ...
    // So, when modifying U(0), remove the coupling.
    if (U(0, 0) > max_voltage(0)) {
      const double overage_amount = U(0, 0) - max_voltage(0);
      mutable_U(0, 0) = max_voltage(0);
      const double coupled_amount =
          (Kff().block<1, 2>(1, 2) * B().block<2, 1>(2, 0))(0, 0) * overage_amount;
      LOG(DEBUG, "Removing coupled amount %f\n", coupled_amount);
      mutable_U(1, 0) += coupled_amount;
    }
    if (U(0, 0) < min_voltage(0)) {
      const double under_amount = U(0, 0) - min_voltage(0);
      mutable_U(0, 0) = min_voltage(0);
      const double coupled_amount =
          (Kff().block<1, 2>(1, 2) * B().block<2, 1>(2, 0))(0, 0) *
          under_amount;
      LOG(DEBUG, "Removing coupled amount %f\n", coupled_amount);
      mutable_U(1, 0) += coupled_amount;
    }

    // Uncapping U above isn't actually a problem with U for the shoulder.
    // Reset any change.
    mutable_U_uncapped(1, 0) = U(1, 0);
    mutable_U(1, 0) =
        ::std::min(max_voltage(1), ::std::max(min_voltage(1), U(1, 0)));
  }

  bool IsAccelerating(double bemf_voltage, double voltage) {
    if (bemf_voltage > 0) {
      return voltage > bemf_voltage;
    } else {
      return voltage < bemf_voltage;
    }
  }
};

class Arm {
 public:
  Arm();
  // Returns whether the estimators have been initialized and zeroed.
  bool initialized() const { return initialized_; }
  bool zeroed() const { return shoulder_zeroed_ && wrist_zeroed_; }
  bool shoulder_zeroed() const { return shoulder_zeroed_; }
  bool wrist_zeroed() const { return wrist_zeroed_; }
  // Returns whether an error has occured
  bool error() const {
    return shoulder_estimator_.error() || wrist_estimator_.error();
  }

  // Updates our estimator with the latest position.
  void Correct(::frc971::PotAndIndexPosition position_shoulder,
               ::frc971::PotAndIndexPosition position_wrist);

  // Forces the goal to be the provided goal.
  void ForceGoal(double unprofiled_goal_shoulder, double unprofiled_goal_wrist);
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_unprofiled_goal(double unprofiled_goal_shoulder,
                           double unprofiled_goal_wrist);

  int controller_index() const { return loop_->controller_index(); }

  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);

  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(double max_angular_velocity_shoulder,
                     double max_angular_acceleration_shoulder,
                     double max_angular_velocity_wrist,
                     double max_angular_acceleration_wrist);
  void set_max_voltage(double shoulder_max_voltage, double wrist_max_voltage);

  void set_shoulder_asymetric_limits(double shoulder_min_voltage,
                                     double shoulder_max_voltage) {
    loop_->set_asymetric_voltage(0, shoulder_min_voltage, shoulder_max_voltage);
  }

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();
  // Resets the internal state.
  void Reset();

  // Returns the current internal estimator state for logging.
  ::frc971::EstimatorState ShoulderEstimatorState();
  ::frc971::EstimatorState WristEstimatorState();

  // Returns the requested shoulder and wrist voltages.
  double shoulder_voltage() const { return loop_->U(0, 0); }
  double wrist_voltage() const { return loop_->U(1, 0); }

  // Returns the current positions.
  double shoulder_angle() const { return Y_(0, 0); }
  double wrist_angle() const { return Y_(1, 0) + Y_(0, 0); }

  // Returns the controller error.
  const StateFeedbackLoop<6, 2, 2> &controller() const { return *loop_; }

  // Returns the unprofiled goal.
  const Eigen::Matrix<double, 6, 1> &unprofiled_goal() const {
    return unprofiled_goal_;
  }
  double unprofiled_goal(int row, int col) const {
    return unprofiled_goal_(row, col);
  }

  // Returns the filtered goal.
  const Eigen::Matrix<double, 6, 1> &goal() const { return loop_->R(); }
  double goal(int row, int col) const { return loop_->R(row, col); }

  // Returns the current state estimate.
  const Eigen::Matrix<double, 6, 1> &X_hat() const { return loop_->X_hat(); }
  double X_hat(int row, int col) const { return loop_->X_hat(row, col); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { shoulder_estimator_.TriggerError(); }

 private:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  void CapGoal(const char *name, Eigen::Matrix<double, 6, 1> *goal);

  // Updates the offset
  void UpdateWristOffset(double offset);
  void UpdateShoulderOffset(double offset);

  friend class testing::SuperstructureTest_DisabledGoalTest_Test;
  ::std::unique_ptr<
      ArmControlLoop> loop_;

  aos::util::TrapezoidProfile shoulder_profile_, wrist_profile_;
  ::frc971::zeroing::ZeroingEstimator shoulder_estimator_, wrist_estimator_;

  // Current measurement.
  Eigen::Matrix<double, 2, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 2, 1> offset_;

  // The goal that the profile tries to reach.
  Eigen::Matrix<double, 6, 1> unprofiled_goal_;

  bool initialized_ = false;
  bool shoulder_zeroed_ = false;
  bool wrist_zeroed_ = false;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_
