#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace frc971 {
namespace control_loops {

class DrivetrainLoop
    : public aos::control_loops::ControlLoop<control_loops::Drivetrain, true, false> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      control_loops::Drivetrain *my_drivetrain = &control_loops::drivetrain)
      : aos::control_loops::ControlLoop<control_loops::Drivetrain, true, false>(
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
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
