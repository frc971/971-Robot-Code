#ifndef Y2023_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_PIVOT_JOINT_PIVOT_JOINT_H_
#define Y2023_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_PIVOT_JOINT_PIVOT_JOINT_H_

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023_bot3/constants.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {

class PivotJoint {
  using PotAndAbsoluteEncoderSubsystem =
      ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

 public:
  PivotJoint(std::shared_ptr<const constants::Values> values);

  flatbuffers::Offset<
      frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
  RunIteration(PivotGoal goal, double *output,
               const frc971::PotAndAbsolutePosition *position,
               flatbuffers::FlatBufferBuilder *status_fbb);

  bool zeroed() const { return pivot_joint_.zeroed(); }

  bool estopped() const { return pivot_joint_.estopped(); }

  // variable which records the last time at which "intake" button was pressed
  aos::monotonic_clock::time_point timer_;

 private:
  PotAndAbsoluteEncoderSubsystem pivot_joint_;
};

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3

#endif  // Y2023_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_PIVOT_JOINT_PIVOT_JOINT_H_
