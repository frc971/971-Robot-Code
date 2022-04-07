#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_

#include "aos/flatbuffers.h"
#include "frc971/control_loops/aiming/aiming.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2022/constants.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022::control_loops::superstructure::turret {

// This class manages taking in drivetrain status messages and generating turret
// goals so that it gets aimed at the goal.
class Aimer {
 public:
  typedef frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      Goal;
  typedef frc971::control_loops::drivetrain::Status Status;
  typedef frc971::control_loops::aiming::ShotMode ShotMode;

  Aimer(std::shared_ptr<const constants::Values> constants);

  void Update(const Status *status, ShotMode shot_mode);

  const Goal *TurretGoal() const { return &goal_.message(); }

  // Returns the distance to the goal, in meters.
  double DistanceToGoal() const { return current_goal_.virtual_shot_distance; }

  flatbuffers::Offset<AimerStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  std::shared_ptr<const constants::Values> constants_;
  aos::FlatbufferDetachedBuffer<Goal> goal_;
  frc971::control_loops::aiming::TurretGoal current_goal_;
};

}  // namespace y2022::control_loops::superstructure::turret
#endif  // Y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
