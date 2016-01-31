#ifndef Y2014_ACTIONS_DRIVETRAIN_ACTION_H_
#define Y2014_ACTIONS_DRIVETRAIN_ACTION_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "y2014/actors/drivetrain_action.q.h"

namespace y2014 {
namespace actors {

class DrivetrainActor
    : public ::aos::common::actions::ActorBase<DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainActor(DrivetrainActionQueueGroup* s);

  bool RunAction(const actors::DrivetrainActionParams &params) override;

 private:
  StateFeedbackLoop<4, 2, 2> loop_;
};

typedef ::aos::common::actions::TypedAction<DrivetrainActionQueueGroup>
    DrivetrainAction;

// Makes a new DrivetrainActor action.
::std::unique_ptr<DrivetrainAction> MakeDrivetrainAction(
    const ::y2014::actors::DrivetrainActionParams& params);

}  // namespace actors
}  // namespace y2014

#endif
