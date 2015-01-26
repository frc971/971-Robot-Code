#ifndef FRC971_CONTROL_LOOPS_FRIDGE_H_
#define FRC971_CONTROL_LOOPS_FRIDGE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/control_loops/fridge/arm_motor_plant.h"
#include "frc971/control_loops/fridge/elevator_motor_plant.h"

namespace frc971 {
namespace control_loops {

class Fridge
    : public aos::controls::ControlLoop<control_loops::FridgeQueue> {
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

 protected:
  virtual void RunIteration(
      const control_loops::FridgeQueue::Goal *goal,
      const control_loops::FridgeQueue::Position *position,
      control_loops::FridgeQueue::Output *output,
      control_loops::FridgeQueue::Status *status);

 private:
  // The state feedback control loop or loops to talk to.
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> left_arm_loop_;
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> right_arm_loop_;
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> left_elev_loop_;
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> right_elev_loop_;
};

}  // namespace control_loops
}  // namespace frc971

#endif // FRC971_CONTROL_LOOPS_FRIDGE_H_

