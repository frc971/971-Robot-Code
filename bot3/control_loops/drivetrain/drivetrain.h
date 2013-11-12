#ifndef BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
#define BOT3_CONTROL_LOOPS_DRIVETRAIN_H_

#include "aos/common/control_loop/ControlLoop.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"

namespace bot3 {
namespace control_loops {

class DrivetrainLoop
    : public aos::control_loops::ControlLoop<control_loops::Drivetrain, false> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      control_loops::Drivetrain *my_drivetrain = &::bot3::control_loops::drivetrain)
      : aos::control_loops::ControlLoop<control_loops::Drivetrain, false>(
          my_drivetrain) {}

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const control_loops::Drivetrain::Goal *goal,
      const control_loops::Drivetrain::Position *position,
      control_loops::Drivetrain::Output *output,
      control_loops::Drivetrain::Status *status);
};

}  // namespace control_loops
}  // namespace bot3

#endif  // BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
