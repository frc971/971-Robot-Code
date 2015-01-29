#ifndef FRC971_ACTIONS_DRIVETRAIN_ACTION_H_
#define FRC971_ACTIONS_DRIVETRAIN_ACTION_H_

#include <memory>

#include "frc971/actions/drivetrain_action.q.h"
#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

namespace frc971 {
namespace actions {

class DrivetrainActor
    : public aos::common::actions::ActorBase<DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainActor(DrivetrainActionQueueGroup* s);

  void RunAction() override;
};

// Makes a new DrivetrainActor action.
::std::unique_ptr<aos::common::actions::TypedAction<DrivetrainActionQueueGroup>>
    MakeDrivetrainAction();

}  // namespace actions
}  // namespace frc971

#endif
