#ifndef Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "frc971/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "y2019/constants.h"
#include "y2019/control_loops/superstructure/collision_avoidance.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"

namespace y2019 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<SuperstructureQueue> {
 public:
  explicit Superstructure(
      ::aos::EventLoop *event_loop,
      const ::std::string &name =
          ".y2019.control_loops.superstructure.superstructure_queue");

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

 protected:
  virtual void RunIteration(const SuperstructureQueue::Goal *unsafe_goal,
                            const SuperstructureQueue::Position *position,
                            SuperstructureQueue::Output *output,
                            SuperstructureQueue::Status *status) override;

 private:
  PotAndAbsoluteEncoderSubsystem elevator_;
  PotAndAbsoluteEncoderSubsystem wrist_;
  AbsoluteEncoderSubsystem intake_;
  PotAndAbsoluteEncoderSubsystem stilts_;

  CollisionAvoidance collision_avoidance_;

  static constexpr double kMinIntakeAngleForRollers = -0.7;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
