#ifndef FRC971_CONTROL_LOOPS_WRIST_H_
#define FRC971_CONTROL_LOOPS_WRIST_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/wrist_motor.q.h"
#include "frc971/control_loops/wrist_motor_plant.h"

namespace frc971 {
namespace control_loops {

class WristMotor
    : public aos::control_loops::ControlLoop<control_loops::WristLoop> {
 public:
  explicit WristMotor(
      control_loops::WristLoop *my_wrist = &control_loops::wrist);

 protected:
  virtual void RunIteration(
      const ::aos::control_loops::Goal *goal,
      const control_loops::WristLoop::Position *position,
      ::aos::control_loops::Output *output,
      ::aos::control_loops::Status *status);

 private:
  ::std::unique_ptr<StateFeedbackLoop<2, 1, 1>> loop_;
  bool stop_;
  int error_count_;
  bool zeroed_;
  double zero_offset_;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_WRIST_H_
