#ifndef FRC971_ACTIONS_DRIVETRAIN_ACTION_H_
#define FRC971_ACTIONS_DRIVETRAIN_ACTION_H_

#include <memory>

#include "frc971/actions/drivetrain_action.q.h"
#include "frc971/actions/action.h"
#include "frc971/actions/action_client.h"

namespace frc971 {
namespace actions {

class DrivetrainAction : public ActionBase<actions::DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainAction(actions::DrivetrainActionQueueGroup* s);

  virtual void RunAction();
};

// Makes a new DrivetrainAction action.
::std::unique_ptr<TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
MakeDrivetrainAction();

}  // namespace actions
}  // namespace frc971

#endif
