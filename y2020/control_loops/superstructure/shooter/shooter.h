#ifndef Y2020_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
#define Y2020_CONTROL_LOOPS_SHOOTER_SHOOTER_H_

#include "frc971/control_loops/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2020/control_loops/superstructure/shooter/flywheel_controller.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_output_generated.h"
#include "y2020/control_loops/superstructure/superstructure_position_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace control_loops {
namespace superstructure {
namespace shooter {

// Handles all flywheels together.
class Shooter {
 public:
  Shooter();

  flatbuffers::Offset<ShooterStatus> RunIteration(
      const ShooterGoal *goal, const ShooterPosition *position,
      flatbuffers::FlatBufferBuilder *fbb, OutputT *output,
      const aos::monotonic_clock::time_point position_timestamp);

  bool ready() { return ready_; }

  float finisher_goal() const { return finisher_.goal(); }
  float accelerator_goal() const { return accelerator_left_.goal(); }

 private:
  FlywheelController finisher_, accelerator_left_, accelerator_right_;

  bool UpToSpeed(const ShooterGoal *goal);
  bool ready_ = false;

  DISALLOW_COPY_AND_ASSIGN(Shooter);
};

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2020

#endif  // Y2020_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
