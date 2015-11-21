#ifndef Y2015_BOT3_CONTROL_LOOPS_INTAKE_H_
#define Y2015_BOT3_CONTROL_LOOPS_INTAKE_H_

#include "aos/common/controls/control_loop.h"

#include "y2015_bot3/control_loops/intake/intake.q.h"

namespace y2015_bot3 {
namespace control_loops {

constexpr double kIntakeVoltageFullPower = 12.0;

class Intake : public aos::controls::ControlLoop<control_loops::IntakeQueue> {
 public:
  explicit Intake(
      control_loops::IntakeQueue *intake_queue = &control_loops::intake_queue);

 protected:
  void RunIteration(const control_loops::IntakeQueue::Goal *goal,
                    const control_loops::IntakeQueue::Position *position,
                    control_loops::IntakeQueue::Output *output,
                    control_loops::IntakeQueue::Status *status) override;
};

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_INTAKE_H_
