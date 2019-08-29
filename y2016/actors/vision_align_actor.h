#ifndef Y2016_ACTORS_DRIVETRAIN_ACTOR_H_
#define Y2016_ACTORS_DRIVETRAIN_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2016/actors/vision_align_action_generated.h"
#include "y2016/vision/vision_generated.h"

namespace y2016 {
namespace actors {

class VisionAlignActor
    : public ::aos::common::actions::ActorBase<vision_align_action::Goal> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<vision_align_action::Goal>
      Factory;

  explicit VisionAlignActor(::aos::EventLoop *event_loop);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/vision_align_action");
  }

  bool RunAction(
      const vision_align_action::VisionAlignActionParams *params) override;

 private:
  ::aos::Fetcher<::y2016::vision::VisionStatus> vision_status_fetcher_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_sender_;
};

}  // namespace actors
}  // namespace y2016

#endif
