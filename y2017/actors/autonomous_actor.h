#ifndef Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2017_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"

namespace y2017 {
namespace actors {
using ::frc971::ProfileParameters;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::frc971::autonomous::AutonomousActionQueueGroup *s);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams &params) override;
 private:
  // TODO(phil): Implement this.
};

}  // namespace actors
}  // namespace y2017

#endif  // Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
