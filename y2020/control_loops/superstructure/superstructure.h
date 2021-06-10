#ifndef y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/input/joystick_state_generated.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/climber.h"
#include "y2020/control_loops/superstructure/shooter/shooter.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"
#include "y2020/control_loops/superstructure/superstructure_position_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"
#include "y2020/control_loops/superstructure/turret/aiming.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  // Terms to control the velocity gain for the friction compensation, and the
  // voltage cap.
  static constexpr double kTurretFrictionGain = 10.0;
  static constexpr double kTurretFrictionVoltageLimit = 1.5;

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;
  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;
  using AbsoluteAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::
              AbsoluteAndAbsoluteEncoderProfiledJointStatus>;

  const AbsoluteAndAbsoluteEncoderSubsystem &hood() const { return hood_; }
  const AbsoluteEncoderSubsystem &intake_joint() const { return intake_joint_; }
  const PotAndAbsoluteEncoderSubsystem &turret() const { return turret_; }
  const shooter::Shooter &shooter() const { return shooter_; }
  double robot_speed() const;

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  AbsoluteAndAbsoluteEncoderSubsystem hood_;
  AbsoluteEncoderSubsystem intake_joint_;
  PotAndAbsoluteEncoderSubsystem turret_;
  shooter::Shooter shooter_;
  turret::Aimer aimer_;

  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  Climber climber_;

  aos::monotonic_clock::time_point shooting_start_time_ =
      aos::monotonic_clock::min_time;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

#endif  // y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
