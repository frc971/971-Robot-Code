#ifndef FRC971_CONTROL_LOOPS_CLAW_H_
#define FRC971_CONTROL_LOOPS_CLAW_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw_motor_plant.h"

namespace frc971 {
namespace control_loops {

class Claw
    : public aos::controls::ControlLoop<control_loops::ClawQueue> {
 public:
  explicit Claw(
      control_loops::ClawQueue *claw_queue = &control_loops::claw_queue);

  // Control loop time step.
  // Please figure out how to set dt from a common location
  // Please decide the correct value
  // Please use dt in your implementation so we can change looptimnig
  // and be consistent with legacy
  // And Brian please approve my code review as people are wait on
  // these files to exist and they will be rewritten anyway
  //static constexpr double dt;

 protected:
  virtual void RunIteration(
      const control_loops::ClawQueue::Goal *goal,
      const control_loops::ClawQueue::Position *position,
      control_loops::ClawQueue::Output *output,
      control_loops::ClawQueue::Status *status);

 private:
  // The state feedback control loop to talk to.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> claw_loop_;
};

}  // namespace control_loops
}  // namespace frc971

#endif // FRC971_CONTROL_LOOPS_CLAW_H_

