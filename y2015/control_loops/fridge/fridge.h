#ifndef Y2015_CONTROL_LOOPS_FRIDGE_H_
#define Y2015_CONTROL_LOOPS_FRIDGE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2015/control_loops/fridge/fridge.q.h"
#include "frc971/zeroing/zeroing.h"
#include "y2015/util/kinematics.h"

namespace y2015 {
namespace control_loops {
namespace fridge {
namespace testing {
class FridgeTest_DisabledGoalTest_Test;
class FridgeTest_ArmGoalPositiveWindupTest_Test;
class FridgeTest_ElevatorGoalPositiveWindupTest_Test;
class FridgeTest_ArmGoalNegativeWindupTest_Test;
class FridgeTest_ElevatorGoalNegativeWindupTest_Test;
class FridgeTest_SafeArmZeroing_Test;
}  // namespace testing

template <int S>
class CappedStateFeedbackLoop : public StateFeedbackLoop<S, 2, 2> {
 public:
  CappedStateFeedbackLoop(StateFeedbackLoop<S, 2, 2> &&loop)
      : StateFeedbackLoop<S, 2, 2>(::std::move(loop)), max_voltage_(12.0) {}

  void set_max_voltage(double max_voltage) {
    max_voltage_ = ::std::max(0.0, ::std::min(12.0, max_voltage));
  }

  void CapU() override;

  // Returns the amount to change the position goals (average and difference) in
  // order to no longer saturate the controller.
  Eigen::Matrix<double, 2, 1> UnsaturateOutputGoalChange();

 private:
  double max_voltage_;
};

class Fridge : public aos::controls::ControlLoop<FridgeQueue> {
 public:
  explicit Fridge(FridgeQueue *fridge_queue =
                      &::y2015::control_loops::fridge::fridge_queue);

  enum State {
    // Waiting to receive data before doing anything.
    UNINITIALIZED = 0,
    // Estimating the starting location.
    INITIALIZING = 1,
    // Moving the elevator to find an index pulse.
    ZEROING_ELEVATOR = 2,
    // Moving the arm to find an index pulse.
    ZEROING_ARM = 3,
    // All good!
    RUNNING = 4,
    // Internal error caused the fridge to abort.
    ESTOP = 5,
  };

  enum class ProfilingType : int32_t {
    // Use angle/height to specify the fridge goal.
    ANGLE_HEIGHT_PROFILING = 0,
    // Use x/y co-ordinates to specify the fridge goal.
    X_Y_PROFILING = 1,
  };

  State state() const { return state_; }

 protected:
  void RunIteration(const FridgeQueue::Goal *goal,
                    const FridgeQueue::Position *position,
                    FridgeQueue::Output *output,
                    FridgeQueue::Status *status) override;

 private:
  friend class testing::FridgeTest_DisabledGoalTest_Test;
  friend class testing::FridgeTest_ElevatorGoalPositiveWindupTest_Test;
  friend class testing::FridgeTest_ArmGoalPositiveWindupTest_Test;
  friend class testing::FridgeTest_ElevatorGoalNegativeWindupTest_Test;
  friend class testing::FridgeTest_ArmGoalNegativeWindupTest_Test;
  friend class testing::FridgeTest_SafeArmZeroing_Test;

  // Sets state_ to the correct state given the current state of the zeroing
  // estimators.
  void UpdateZeroingState();

  void SetElevatorOffset(double left_offset, double right_offset);
  void SetArmOffset(double left_offset, double right_offset);

  // Getters for the current elevator positions.
  double left_elevator();
  double right_elevator();
  double elevator();

  // Getters for the current arm positions.
  double left_arm();
  double right_arm();
  double arm();

  // Our best guess at the current position of the elevator.
  double estimated_left_elevator();
  double estimated_right_elevator();
  double estimated_elevator();

  // Our best guess at the current position of the arm.
  double estimated_left_arm();
  double estimated_right_arm();
  double estimated_arm();

  // Returns the current zeroing velocity for either subsystem.
  // If the subsystem is too far away from the center, these will switch
  // directions.
  double elevator_zeroing_velocity();
  double arm_zeroing_velocity();

  // Corrects the Observer with the current position.
  void Correct();

  double UseUnlessZero(double target_value, double default_value);

  // The state feedback control loop or loops to talk to.
  ::std::unique_ptr<CappedStateFeedbackLoop<5>> arm_loop_;
  ::std::unique_ptr<CappedStateFeedbackLoop<4>> elevator_loop_;

  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator left_arm_estimator_;
  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator right_arm_estimator_;
  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator left_elevator_estimator_;
  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator right_elevator_estimator_;

  // Offsets from the encoder position to the absolute position.  Add these to
  // the encoder position to get the absolute position.
  double left_elevator_offset_ = 0.0;
  double right_elevator_offset_ = 0.0;
  double left_arm_offset_ = 0.0;
  double right_arm_offset_ = 0.0;

  // Current velocity to move at while zeroing.
  double elevator_zeroing_velocity_ = 0.0;
  double arm_zeroing_velocity_ = 0.0;

  // The goals for the elevator and arm.
  double elevator_goal_ = 0.0;
  double arm_goal_ = 0.0;

  double arm_goal_velocity_ = 0.0;
  double elevator_goal_velocity_ = 0.0;

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  FridgeQueue::Position current_position_;

  ProfilingType last_profiling_type_;
  aos::util::ElevatorArmKinematics kinematics_;

  aos::util::TrapezoidProfile arm_profile_;
  aos::util::TrapezoidProfile elevator_profile_;

  aos::util::TrapezoidProfile x_profile_;
  aos::util::TrapezoidProfile y_profile_;
};

}  // namespace fridge
}  // namespace control_loops
}  // namespace y2015

#endif  // Y2015_CONTROL_LOOPS_FRIDGE_H_
