#ifndef BOT3_CONTROL_LOOPS_INTAKE_H_
#define BOT3_CONTROL_LOOPS_INTAKE_H_

#include "aos/common/controls/control_loop.h"

#include "bot3/control_loops/intake/intake.q.h"

namespace bot3 {
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
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_INTAKE_H_
