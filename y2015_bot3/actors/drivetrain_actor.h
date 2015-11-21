#ifndef Y2015_BOT3_ACTIONS_DRIVETRAIN_ACTION_H_
#define Y2015_BOT3_ACTIONS_DRIVETRAIN_ACTION_H_

#include <memory>

#include "y2015_bot3/actors/drivetrain_action.q.h"
#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

namespace frc971 {
namespace actors {

class DrivetrainActor
    : public aos::common::actions::ActorBase<DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainActor(DrivetrainActionQueueGroup* s);

  bool RunAction(const actors::DrivetrainActionParams &params) override;
};

typedef aos::common::actions::TypedAction<DrivetrainActionQueueGroup>
    DrivetrainAction;

// Makes a new DrivetrainActor action.
::std::unique_ptr<DrivetrainAction> MakeDrivetrainAction(
    const ::frc971::actors::DrivetrainActionParams& params);

}  // namespace actors
}  // namespace frc971

#endif  // Y2015_BOT3_ACTIONS_DRIVETRAIN_ACTION_H_
