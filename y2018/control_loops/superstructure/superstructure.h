#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2018/control_loops/superstructure/arm/arm.h"
#include "y2018/control_loops/superstructure/intake/intake.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<control_loops::SuperstructureQueue> {
 public:
  explicit Superstructure(
      control_loops::SuperstructureQueue *my_superstructure =
          &control_loops::superstructure_queue);

  const intake::IntakeSide &intake_left() const { return intake_left_; }
  const intake::IntakeSide &intake_right() const { return intake_right_; }
  const arm::Arm &arm() const { return arm_; }

 protected:
  virtual void RunIteration(
      const control_loops::SuperstructureQueue::Goal *unsafe_goal,
      const control_loops::SuperstructureQueue::Position *position,
      control_loops::SuperstructureQueue::Output *output,
      control_loops::SuperstructureQueue::Status *status) override;

 private:
  intake::IntakeSide intake_left_;
  intake::IntakeSide intake_right_;
  arm::Arm arm_;

  enum class RotationState {
    NOT_ROTATING = 0,
    ROTATING_LEFT = 1,
    ROTATING_RIGHT = 2
  };

  RotationState rotation_state_ = RotationState::NOT_ROTATING;
  int rotation_count_ = 0;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
