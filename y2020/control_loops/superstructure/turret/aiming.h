#ifndef y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
#define y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_

#include "aos/flatbuffers.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {

// This class manages taking in drivetrain status messages and generating turret
// goals so that it gets aimed at the goal.
class Aimer {
 public:
  typedef frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      Goal;
  typedef frc971::control_loops::drivetrain::Status Status;
  Aimer();
  void Update(const Status *status);
  const Goal *TurretGoal() const { return &goal_.message(); }

  flatbuffers::Offset<AimerStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  aos::FlatbufferDetachedBuffer<Goal> goal_;
};

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
#endif  // y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
