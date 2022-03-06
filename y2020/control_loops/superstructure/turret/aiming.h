#ifndef y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
#define y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_

#include "aos/flatbuffers.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/control_loops/aiming/aiming.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace turret {

// Returns the port that we want to score on given our current alliance. The yaw
// of the port will be such that the positive x axis points out the back of the
// target.
frc971::control_loops::Pose InnerPortPose(aos::Alliance alliance);
frc971::control_loops::Pose OuterPortPose(aos::Alliance alliance);

// This class manages taking in drivetrain status messages and generating turret
// goals so that it gets aimed at the goal.
class Aimer {
 public:
  typedef frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      Goal;
  typedef frc971::control_loops::drivetrain::Status Status;
  // Mode to run the aimer in, to control how we manage wrapping the turret
  // angle.
  enum class WrapMode {
    // Keep the turret as far away from the edges of the range of motion as
    // reasonable, to minimize the odds that we will hit the hardstops once we
    // start shooting.
    kAvoidEdges,
    // Do everything reasonable to avoid having to wrap the shooter--set this
    // while shooting so that we don't randomly spin the shooter 360 while
    // shooting.
    kAvoidWrapping,
  };

  typedef frc971::control_loops::aiming::ShotMode ShotMode;

  Aimer();

  void Update(const Status *status, aos::Alliance alliance, WrapMode wrap_mode,
              ShotMode shot_mode);

  const Goal *TurretGoal() const { return &goal_.message(); }

  // Returns the distance to the goal, in meters.
  double DistanceToGoal() const { return shot_distance_; }

  flatbuffers::Offset<AimerStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) const;

 private:
  aos::FlatbufferDetachedBuffer<Goal> goal_;
  bool aiming_for_inner_port_ = false;
  // Distance of the shot to the virtual target, used for calculating hood
  // position and shooter speed.
  double shot_distance_ = 0.0;  // meters
  // Real-world distance to the target.
  double target_distance_ = 0.0;  // meters
  double inner_port_angle_ = 0.0;  // radians
};

}  // namespace turret
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020
#endif  // y2020_CONTROL_LOOPS_SUPERSTRUCTURE_TURRET_AIMING_H_
