#ifndef Y2015_CONTROL_LOOPS_CLAW_H_
#define Y2015_CONTROL_LOOPS_CLAW_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/control_loops/claw/claw_motor_plant.h"
#include "frc971/zeroing/zeroing.h"

namespace y2015 {
namespace control_loops {
namespace testing {
class ClawTest_DisabledGoal_Test;
class ClawTest_GoalPositiveWindup_Test;
class ClawTest_GoalNegativeWindup_Test;
}  // namespace testing

class ClawCappedStateFeedbackLoop : public StateFeedbackLoop<2, 1, 1> {
 public:
  ClawCappedStateFeedbackLoop(StateFeedbackLoop<2, 1, 1> &&loop)
      : StateFeedbackLoop<2, 1, 1>(::std::move(loop)), max_voltage_(12.0) {}

  void set_max_voltage(double max_voltage) {
    max_voltage_ = ::std::max(0.0, ::std::min(12.0, max_voltage));
  }

  void CapU() override;

  // Returns the amount to change the position goal in order to no longer
  // saturate the controller.
  double UnsaturateOutputGoalChange();

 private:
  double max_voltage_;
};

class Claw : public aos::controls::ControlLoop<control_loops::ClawQueue> {
 public:
  enum State {
    // Waiting to receive data before doing anything.
    UNINITIALIZED = 0,
    // Estimating the starting location.
    INITIALIZING = 1,
    // Moving to find an index pulse.
    ZEROING = 2,
    // Normal operation.
    RUNNING = 3,
    // Internal error caused the claw to abort.
    ESTOP = 4,
  };

  int state() { return state_; }

  explicit Claw(
      control_loops::ClawQueue *claw_queue = &control_loops::claw_queue);

 protected:
  virtual void RunIteration(const control_loops::ClawQueue::Goal *goal,
                            const control_loops::ClawQueue::Position *position,
                            control_loops::ClawQueue::Output *output,
                            control_loops::ClawQueue::Status *status);

 private:
  friend class testing::ClawTest_DisabledGoal_Test;
  friend class testing::ClawTest_GoalPositiveWindup_Test;
  friend class testing::ClawTest_GoalNegativeWindup_Test;

  // Sets state_ to the correct state given the current state of the zeroing
  // estimator.
  void UpdateZeroingState();
  void SetClawOffset(double offset);
  // Corrects the Observer with the current position.
  void Correct();

  // Getter for the current claw position.
  double claw_position() const;
  // Our best guess at the current position of the claw.
  double estimated_claw_position() const;
  // Returns the current zeroing velocity for the claw. If the subsystem is too
  // far away from the center, it will switch directions.
  double claw_zeroing_velocity();

  State state_ = UNINITIALIZED;

  // The time when we last changed the claw piston state.
  ::aos::monotonic_clock::time_point last_piston_edge_;

  // The state feedback control loop to talk to.
  ::std::unique_ptr<ClawCappedStateFeedbackLoop> claw_loop_;

  // Latest position from queue.
  control_loops::ClawQueue::Position current_position_;
  // Zeroing estimator for claw.
  ::frc971::zeroing::PotAndIndexPulseZeroingEstimator claw_estimator_;

  // The goal for the claw.
  double claw_goal_ = 0.0;
  // Current velocity to move at while zeroing.
  double claw_zeroing_velocity_ = 0.0;
  // Offsets from the encoder position to the absolute position.
  double claw_offset_ = 0.0;

  // Whether claw was closed last cycle.
  bool last_rollers_closed_ = false;

  ::aos::util::TrapezoidProfile profile_;
};

}  // namespace control_loops
}  // namespace y2015

#endif  // Y2015_CONTROL_LOOPS_CLAW_H_
