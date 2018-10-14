#ifndef Y2017_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2017_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/controls/control_loop.h"
#include "aos/util/trapezoid_profile.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/zeroing/zeroing.h"
#include "y2017_bot3/control_loops/superstructure/superstructure.q.h"

namespace y2017_bot3 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  static constexpr double kOperatingVoltage = 12.0;

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position * /*position*/,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status * /*status*/) override;

 private:
  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017_bot3

#endif  // Y2017_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
