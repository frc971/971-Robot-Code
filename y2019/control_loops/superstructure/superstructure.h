#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<SuperstructureQueue> {
 public:
  explicit Superstructure(
      SuperstructureQueue *my_superstructure =
          &superstructure_queue);

 protected:
  virtual void RunIteration(
      const SuperstructureQueue::Goal *unsafe_goal,
      const SuperstructureQueue::Position *position,
      SuperstructureQueue::Output *output,
      SuperstructureQueue::Status *status) override;

 private:

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_