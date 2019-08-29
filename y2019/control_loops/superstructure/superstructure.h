#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2019/constants.h"
#include "y2019/control_loops/superstructure/collision_avoidance.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_output_generated.h"
#include "y2019/control_loops/superstructure/superstructure_position_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"
#include "y2019/control_loops/superstructure/vacuum.h"
#include "y2019/status_light_generated.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;
  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  const PotAndAbsoluteEncoderSubsystem &elevator() const { return elevator_; }
  const PotAndAbsoluteEncoderSubsystem &wrist() const { return wrist_; }
  const AbsoluteEncoderSubsystem &intake() const { return intake_; }
  const PotAndAbsoluteEncoderSubsystem &stilts() const { return stilts_; }
  const Vacuum &vacuum() const { return vacuum_; }

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  void SendColors(float red, float green, float blue);

  ::aos::Sender<::y2019::StatusLight> status_light_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  PotAndAbsoluteEncoderSubsystem elevator_;
  PotAndAbsoluteEncoderSubsystem wrist_;
  AbsoluteEncoderSubsystem intake_;
  PotAndAbsoluteEncoderSubsystem stilts_;
  Vacuum vacuum_;

  CollisionAvoidance collision_avoidance_;

  int line_blink_count_ = 0;

  static constexpr double kMinIntakeAngleForRollers = -0.7;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
