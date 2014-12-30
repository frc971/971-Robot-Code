#ifndef BOT3_CONTROL_LOOPS_ROLLERS_H_
#define BOT3_CONTROL_LOOPS_ROLLERS_H_

#include "aos/common/controls/control_loop.h"
#include "bot3/control_loops/rollers/rollers.q.h"

namespace bot3 {
namespace control_loops {

class RollersLoop
  : public aos::controls::ControlLoop<control_loops::Rollers,
      false, false, true> {
 public:
  // Constructs a control loops which can take a rollers or defaults to the
  // rollers at ::bot3::control_loops::rollers.
  explicit RollersLoop(
      control_loops::Rollers *my_rollers = &control_loops::rollers)
      : aos::controls::ControlLoop<control_loops::Rollers, false, false, true>(
          my_rollers) {}

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const control_loops::Rollers::Goal *goal,
      const control_loops::Rollers::Position *position,
      control_loops::Rollers::Output *output,
      control_loops::Rollers::Status *status);
};

}  // namespace control_loops
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_ROLLERS_H_
