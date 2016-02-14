#ifndef Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/zeroing/zeroing.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/control_loops/superstructure/superstructure_controls.h"

namespace y2016 {
namespace control_loops {
namespace superstructure {
namespace testing {
class SuperstructureTest_DisabledGoalTest_Test;
}  // namespace testing

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);
  enum State {
    // Waiting to receive data before doing anything.
    UNINITIALIZED = 0,
    // Estimating the starting location.
    INITIALIZING = 1,
    // Moving the intake to find an index pulse.
    ZEROING_INTAKE = 2,
    // Moving the arm to find an index pulse.
    ZEROING_ARM = 3,
    // All good!
    RUNNING = 4,
    // Internal error caused the superstructure to abort.
    ESTOP = 5,
  };

  State state() const { return state_; }

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  friend class testing::SuperstructureTest_DisabledGoalTest_Test;
  Intake intake_;
  Arm arm_;

  State state_ = UNINITIALIZED;
  State last_state_ = UNINITIALIZED;

  // Sets state_ to the correct state given the current state of the zeroing
  // estimators.
  void UpdateZeroingState();

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
