#ifndef Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_
#define Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/profiled_subsystem.h"
#include "frc971/control_loops/simple_capped_state_feedback_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016/control_loops/superstructure/integral_arm_plant.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {
namespace testing {
class SuperstructureTest_DisabledGoalTest_Test;
}  // namespace testing

class ArmControlLoop
    : public ::frc971::control_loops::SimpleCappedStateFeedbackLoop<6, 2, 2> {
 public:
  ArmControlLoop(SimpleCappedStateFeedbackLoop<6, 2, 2> &&loop)
      : SimpleCappedStateFeedbackLoop<6, 2, 2>(::std::move(loop)) {}

  const Eigen::Matrix<double, 2, 1> ControllerOutput() override {
    const Eigen::Matrix<double, 2, 1> accelerating_ff =
        controller().coefficients(0).Kff *
        (next_R() - plant().coefficients(0).A * R());
    const Eigen::Matrix<double, 2, 1> accelerating_controller =
        controller().coefficients(0).K * error() + accelerating_ff;

    const Eigen::Matrix<double, 2, 1> decelerating_ff =
        controller().coefficients(1).Kff *
        (next_R() - plant().coefficients(1).A * R());
    const Eigen::Matrix<double, 2, 1> decelerating_controller =
        controller().coefficients(1).K * error() + decelerating_ff;

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
      set_index(0);
      ff_U_ = accelerating_ff;
      return accelerating_controller;
    } else {
      set_index(1);
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
      const double coupled_amount = (controller().Kff().block<1, 2>(1, 2) *
                                     plant().B().block<2, 1>(2, 0))(0, 0) *
                                    overage_amount;
      LOG(DEBUG, "Removing coupled amount %f\n", coupled_amount);
      mutable_U(1, 0) += coupled_amount;
    }
    if (U(0, 0) < min_voltage(0)) {
      const double under_amount = U(0, 0) - min_voltage(0);
      mutable_U(0, 0) = min_voltage(0);
      const double coupled_amount = (controller().Kff().block<1, 2>(1, 2) *
                                     plant().B().block<2, 1>(2, 0))(0, 0) *
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

class Intake : public ::frc971::control_loops::SingleDOFProfiledSubsystem<> {
 public:
  Intake();
};


class Arm : public ::frc971::control_loops::ProfiledSubsystem<6, 2> {
 public:
  Arm();

  // Updates our estimator with the latest position.
  void Correct(::frc971::PotAndIndexPosition position_shoulder,
               ::frc971::PotAndIndexPosition position_wrist);

  // Forces the goal to be the provided goal.
  void ForceGoal(double unprofiled_goal_shoulder, double unprofiled_goal_wrist);
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_unprofiled_goal(double unprofiled_goal_shoulder,
                           double unprofiled_goal_wrist);

  // Runs the controller and profile generator for a cycle.
  void Update(bool disabled);

  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(double max_angular_velocity_shoulder,
                     double max_angular_acceleration_shoulder,
                     double max_angular_velocity_wrist,
                     double max_angular_acceleration_wrist);

  void set_shoulder_asymetric_limits(double shoulder_min_voltage,
                                     double shoulder_max_voltage) {
    loop_->set_asymetric_voltage(0, shoulder_min_voltage, shoulder_max_voltage);
  }

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();

  // Returns the requested shoulder and wrist voltages.
  double shoulder_voltage() const { return loop_->U(0, 0); }
  double wrist_voltage() const { return loop_->U(1, 0); }

  // Returns the current positions.
  double shoulder_angle() const { return Y_(0, 0); }
  double wrist_angle() const { return Y_(1, 0) + Y_(0, 0); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { estimators_[0].TriggerError(); }

 private:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  void CapGoal(const char *name, Eigen::Matrix<double, 6, 1> *goal);

  // Updates the offset
  void UpdateWristOffset(double offset);
  void UpdateShoulderOffset(double offset);

  friend class testing::SuperstructureTest_DisabledGoalTest_Test;

  aos::util::TrapezoidProfile shoulder_profile_, wrist_profile_;

  // Current measurement.
  Eigen::Matrix<double, 2, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 2, 1> offset_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_CONTROLS_H_
