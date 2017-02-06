#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  enum State {
    // Wait for all the filters to be ready before starting the initialization
    // process.
    UNINITIALIZED = 0,

    // We now are ready to decide how to zero.  Decide what to do once we are
    // enabled.
    DISABLED_INITIALIZED = 1,

    ZEROING = 2,
    // Run with full power.
    RUNNING = 3,
    // Internal error caused the superstructure to abort.
    ESTOP = 4,
  };

  bool IsRunning() const { return state_ == RUNNING; }

  State state() const { return state_; }

  // Returns the value to move the joint to such that it will stay below
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepBelow(double reference_angle, double current_angle,
                                 double move_distance);
  // Returns the value to move the joint to such that it will stay above
  // reference_angle starting at current_angle, but move at least move_distance
  static double MoveButKeepAbove(double reference_angle, double current_angle,
                                 double move_distance);

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  hood::Hood hood_;

  State state_ = UNINITIALIZED;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
