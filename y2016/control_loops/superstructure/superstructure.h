#ifndef Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016/control_loops/superstructure/superstructure.q.h"

namespace y2016 {
namespace control_loops {

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *goal,
      const control_loops::SuperstructureQueue::Position *position,
      ::aos::control_loops::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace control_loops
}  // namespace y2016

#endif  // Y2016_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
