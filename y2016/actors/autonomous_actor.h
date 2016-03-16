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
using ::frc971::ProfileParameters;

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
                  ::frc971::ProfileParameters linear,
                  ::frc971::ProfileParameters angular);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();

  const ::frc971::control_loops::drivetrain::DrivetrainConfig dt_config_;

  // Initial drivetrain positions.
  struct InitialDrivetrain {
    double left;
    double right;
  };
  InitialDrivetrain initial_drivetrain_;

  // Internal struct holding superstructure goals sent by autonomous to the
  // loop.
  struct SuperstructureGoal {
    double intake;
    double shoulder;
    double wrist;
  };
  SuperstructureGoal superstructure_goal_;

  void MoveSuperstructure(double intake, double shoulder, double wrist,
                          const ProfileParameters intake_params,
                          const ProfileParameters shoulder_params,
                          const ProfileParameters wrist_params, double top_rollers,
                          double bottom_rollers);
  void WaitForSuperstructure();
};

typedef ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>
    AutonomousAction;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
