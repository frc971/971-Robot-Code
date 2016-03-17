#ifndef Y2016_ACTORS_DRIVETRAIN_ACTOR_H_
#define Y2016_ACTORS_DRIVETRAIN_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016/actors/vision_align_action.q.h"

namespace y2016 {
namespace actors {

class VisionAlignActor
    : public ::aos::common::actions::ActorBase<VisionAlignActionQueueGroup> {
 public:
  explicit VisionAlignActor(VisionAlignActionQueueGroup *s);

  bool RunAction(const actors::VisionAlignActionParams &params) override;
};

typedef ::aos::common::actions::TypedAction<VisionAlignActionQueueGroup>
    VisionAlignAction;

// Makes a new VisionAlignActor action.
::std::unique_ptr<VisionAlignAction> MakeVisionAlignAction(
    const ::y2016::actors::VisionAlignActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif
