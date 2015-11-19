#ifndef Y2014_BOT3_CONTROL_LOOPS_ROLLERS_H_
#define Y2014_BOT3_CONTROL_LOOPS_ROLLERS_H_

#include "aos/common/controls/control_loop.h"

#include "y2014_bot3/control_loops/rollers/rollers.q.h"

namespace y2014_bot3 {
namespace control_loops {

class Rollers : public aos::controls::ControlLoop<control_loops::RollersQueue> {
 public:
  // Constructs a control loops which can take a rollers or defaults to the
  // rollers at ::2014_bot3::control_loops::rollers.
  explicit Rollers(control_loops::RollersQueue *rollers_queue =
                       &control_loops::rollers_queue);

 protected:
  // Executes one cycle of the control loop.
  void RunIteration(const control_loops::RollersQueue::Goal *goal,
                    const control_loops::RollersQueue::Position *position,
                    control_loops::RollersQueue::Output *output,
                    control_loops::RollersQueue::Status *status) override;
};

}  // namespace control_loops
}  // namespace y2014_bot3

#endif  // Y2014_BOT3_CONTROL_LOOPS_ROLLERS_H_
