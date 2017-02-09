#ifndef Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2017/control_loops/superstructure/hood/hood.h"
#include "y2017/control_loops/superstructure/turret/turret.h"
#include "y2017/control_loops/superstructure/intake/intake.h"
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

  const hood::Hood &hood() const { return hood_; }
  const turret::Turret &turret() const { return turret_; }
  const intake::Intake &intake() const { return intake_; }

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  hood::Hood hood_;
  turret::Turret turret_;
  intake::Intake intake_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
