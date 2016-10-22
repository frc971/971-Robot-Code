#ifndef Y2016_BOT3_ACTORS_DRIVETRAIN_ACTOR_H_
#define Y2016_BOT3_ACTORS_DRIVETRAIN_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2016_bot3/actors/drivetrain_action.q.h"

namespace y2016_bot3 {
namespace actors {

class DrivetrainActor
    : public ::aos::common::actions::ActorBase<DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainActor(DrivetrainActionQueueGroup *s);

  bool RunAction(const actors::DrivetrainActionParams &params) override;

 private:
  StateFeedbackLoop<4, 2, 2> loop_;
};

typedef ::aos::common::actions::TypedAction<DrivetrainActionQueueGroup>
    DrivetrainAction;

// Makes a new DrivetrainActor action.
::std::unique_ptr<DrivetrainAction> MakeDrivetrainAction(
    const ::y2016_bot3::actors::DrivetrainActionParams &params);

}  // namespace actors
}  // namespace y2016_bot3

#endif
