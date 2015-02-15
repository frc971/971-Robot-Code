#ifndef FRC971_CONTROL_LOOPS_FRIDGE_H_
#define FRC971_CONTROL_LOOPS_FRIDGE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/fridge/arm_motor_plant.h"
#include "frc971/control_loops/fridge/elevator_motor_plant.h"
#include "frc971/zeroing/zeroing.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class FridgeTest_DisabledGoalTest_Test;
class FridgeTest_ArmGoalPositiveWindupTest_Test;
class FridgeTest_ElevatorGoalPositiveWindupTest_Test;
class FridgeTest_ArmGoalNegativeWindupTest_Test;
class FridgeTest_ElevatorGoalNegativeWindupTest_Test;
}

class CappedStateFeedbackLoop : public StateFeedbackLoop<4, 2, 2> {
 public:
  CappedStateFeedbackLoop(StateFeedbackLoop<4, 2, 2> &&loop)
      : StateFeedbackLoop<4, 2, 2>(::std::move(loop)), max_voltage_(12.0) {}

  void set_max_voltage(double max_voltage) {
    max_voltage_ = ::std::max(-12.0, ::std::min(12.0, max_voltage));
  }

  void CapU() override;

  // Returns the amount to change the position goals (average and difference) in
  // order to no longer saturate the controller.
  Eigen::Matrix<double, 2, 1> UnsaturateOutputGoalChange();

 private:
  double max_voltage_;
};

class Fridge
    : public aos::controls::ControlLoop<control_loops::FridgeQueue, false> {
 public:
  explicit Fridge(
      control_loops::FridgeQueue *fridge_queue = &control_loops::fridge_queue);

  // Control loop time step.
  // Please figure out how to set dt from a common location
  // Please decide the correct value
  // Please use dt in your implementation so we can change looptimnig
  // and be consistent with legacy
  // And Brian please approve my code review as people are wait on
  // these files to exist and they will be rewritten anyway
  //static constexpr double dt;

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

  State state() const { return state_; }

 protected:
  void RunIteration(const control_loops::FridgeQueue::Goal *goal,
                    const control_loops::FridgeQueue::Position *position,
                    control_loops::FridgeQueue::Output *output,
                    control_loops::FridgeQueue::Status *status) override;

 private:
  friend class testing::FridgeTest_DisabledGoalTest_Test;
  friend class testing::FridgeTest_ElevatorGoalPositiveWindupTest_Test;
  friend class testing::FridgeTest_ArmGoalPositiveWindupTest_Test;
  friend class testing::FridgeTest_ElevatorGoalNegativeWindupTest_Test;
  friend class testing::FridgeTest_ArmGoalNegativeWindupTest_Test;

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

  // The state feedback control loop or loops to talk to.
  ::std::unique_ptr<CappedStateFeedbackLoop> arm_loop_;
  ::std::unique_ptr<CappedStateFeedbackLoop> elevator_loop_;

  zeroing::ZeroingEstimator left_arm_estimator_;
  zeroing::ZeroingEstimator right_arm_estimator_;
  zeroing::ZeroingEstimator left_elevator_estimator_;
  zeroing::ZeroingEstimator right_elevator_estimator_;

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

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  control_loops::FridgeQueue::Position current_position_;
  static constexpr double dt = 0.005;
};

}  // namespace control_loops
}  // namespace frc971

#endif // FRC971_CONTROL_LOOPS_FRIDGE_H_

