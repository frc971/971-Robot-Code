#ifndef Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_ACTORS_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

#include "y2016/actors/autonomous_action.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2016 {
namespace actors {

class AutonomousActor
    : public ::aos::common::actions::ActorBase<AutonomousActionQueueGroup> {
 public:
  explicit AutonomousActor(AutonomousActionQueueGroup *s);

  bool RunAction(const actors::AutonomousActionParams &params) override;

 private:
  void ResetDrivetrain();
  void InitializeEncoders();
  void WaitUntilDoneOrCanceled(::std::unique_ptr<aos::common::actions::Action>
      action);
  void StartDrive(double distance, double angle,
                  ::frc971::control_loops::ProfileParameters linear,
                  ::frc971::control_loops::ProfileParameters angular);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();

  double left_initial_position_, right_initial_position_;

  const ::frc971::control_loops::drivetrain::DrivetrainConfig dt_config_;
};

typedef ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>
    AutonomousAction;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
