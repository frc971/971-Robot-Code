#ifndef Y2014_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2014_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/event_loop.h"
#include "frc971/autonomous/base_autonomous_actor.h"

namespace y2014_bot3 {
namespace actors {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset() {
    InitializeEncoders();
    ResetDrivetrain();
  }
};

}  // namespace actors
}  // namespace y2014_bot3

#endif  // Y2014_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
