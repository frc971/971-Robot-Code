#include "pivot_joint.h"

#include "aos/events/event_loop.h"
#include "aos/time/time.h"
#include "frc971/control_loops/control_loop.h"
#include "y2023_bot3/constants.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023_bot3 {
namespace control_loops {
namespace superstructure {

PivotJoint::PivotJoint(std::shared_ptr<const constants::Values> values)
    : pivot_joint_(values->pivot_joint.subsystem_params) {}

flatbuffers::Offset<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
PivotJoint::RunIteration(PivotGoal goal, double *output,
                         const frc971::PotAndAbsolutePosition *position,
                         flatbuffers::FlatBufferBuilder *status_fbb) {
  double pivot_goal = 0;
  switch (goal) {
    case PivotGoal::NEUTRAL:
      pivot_goal = 0;
      break;

    case PivotGoal::PICKUP_FRONT:
      pivot_goal = 0.25;
      break;

    case PivotGoal::PICKUP_BACK:
      pivot_goal = 0.30;
      break;

    case PivotGoal::SCORE_LOW_FRONT:
      pivot_goal = 0.35;
      break;

    case PivotGoal::SCORE_LOW_BACK:
      pivot_goal = 0.40;
      break;

    case PivotGoal::SCORE_MID_FRONT:
      pivot_goal = 0.45;
      break;

    case PivotGoal::SCORE_MID_BACK:
      pivot_goal = 0.5;
      break;
  }

  flatbuffers::Offset<
      frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal>
      pivot_joint_offset = frc971::control_loops::
          CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *status_fbb, pivot_goal,
              frc971::CreateProfileParameters(*status_fbb, 12.0, 90.0));

  status_fbb->Finish(pivot_joint_offset);

  flatbuffers::Offset<
      frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
      pivot_joint_status_offset = pivot_joint_.Iterate(
          flatbuffers::GetRoot<frc971::control_loops::
                                   StaticZeroingSingleDOFProfiledSubsystemGoal>(
              status_fbb->GetBufferPointer()),
          position, output, status_fbb);

  return pivot_joint_status_offset;
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023_bot3
