#ifndef y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/controls/control_loop.h"
#include "aos/events/event_loop.h"
#include "y2020/constants.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"
#include "y2020/control_loops/superstructure/superstructure_position_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {

class Superstructure
    : public ::aos::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  using AbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::AbsoluteEncoderProfiledJointStatus>;

  const AbsoluteEncoderSubsystem &hood() const { return hood_; }
  const AbsoluteEncoderSubsystem &intake_joint() const { return intake_joint_; }

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  AbsoluteEncoderSubsystem hood_;
  AbsoluteEncoderSubsystem intake_joint_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

#endif  // y2020_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
