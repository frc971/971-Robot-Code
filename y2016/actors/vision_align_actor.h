#ifndef Y2016_ACTORS_DRIVETRAIN_ACTOR_H_
#define Y2016_ACTORS_DRIVETRAIN_ACTOR_H_

#include <memory>

#include "aos/actions/actor.h"
#include "aos/actions/actions.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016/actors/vision_align_action.q.h"

namespace y2016 {
namespace actors {

class VisionAlignActor
    : public ::aos::common::actions::ActorBase<VisionAlignActionQueueGroup> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<
      VisionAlignActionQueueGroup>
      Factory;

  explicit VisionAlignActor(::aos::EventLoop *event_loop);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, ".y2016.actors.vision_align_action");
  }

  bool RunAction(const actors::VisionAlignActionParams &params) override;
};

}  // namespace actors
}  // namespace y2016

#endif
