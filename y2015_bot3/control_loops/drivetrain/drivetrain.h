#ifndef Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
#define Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "aos/common/controls/polytope.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "aos/common/util/log_interval.h"

#include "frc971/shifter_hall_effect.h"
#include "y2015_bot3/control_loops/drivetrain/drivetrain.q.h"

namespace y2015_bot3 {
namespace control_loops {

// Constants
// TODO(comran): Get actual constants.
constexpr double kDrivetrainTurnWidth = 0.63;
constexpr double kDrivetrainEncoderRatio = 18.0 / 44.0;
constexpr double kDrivetrainHighGearRatio =
    kDrivetrainEncoderRatio * 18.0 / 60.0;
constexpr double kDrivetrainLowGearRatio = kDrivetrainHighGearRatio;
const bool kDrivetrainClutchTransmission = false;
const ::frc971::constants::ShifterHallEffect kDrivetrainRightShifter{
    555, 657, 660, 560, 0.2, 0.7};
const ::frc971::constants::ShifterHallEffect kDrivetrainLeftShifter{
    555, 660, 644, 552, 0.2, 0.7};
// End constants

class DrivetrainLoop
    : public aos::controls::ControlLoop<control_loops::DrivetrainQueue> {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at y2015_bot3::control_loops::drivetrain
  explicit DrivetrainLoop(control_loops::DrivetrainQueue *my_drivetrain =
                              &control_loops::drivetrain_queue)
      : aos::controls::ControlLoop<control_loops::DrivetrainQueue>(
            my_drivetrain) {
    ::aos::controls::HPolytope<0>::Init();
  }

 protected:
  // Executes one cycle of the control loop.
  virtual void RunIteration(
      const control_loops::DrivetrainQueue::Goal *goal,
      const control_loops::DrivetrainQueue::Position *position,
      control_loops::DrivetrainQueue::Output *output,
      control_loops::DrivetrainQueue::Status *status);

  typedef ::aos::util::SimpleLogInterval SimpleLogInterval;
  SimpleLogInterval no_position_ = SimpleLogInterval(
      ::aos::time::Time::InSeconds(0.25), WARNING, "no position");
};

}  // namespace control_loops
}  // namespace y2015_bot3

#endif  // Y2015_BOT3_CONTROL_LOOPS_DRIVETRAIN_H_
