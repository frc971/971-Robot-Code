#ifndef BOT3_ACTIONS_DRIVETRAIN_ACTION_H_
#define BOT3_ACTIONS_DRIVETRAIN_ACTION_H_

#include <memory>

#include "frc971/actions/action.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/drivetrain_action.q.h"

namespace bot3 {
namespace actions {

class DrivetrainAction : public
    ::frc971::actions::ActionBase<::frc971::actions::DrivetrainActionQueueGroup> {
 public:
  explicit DrivetrainAction(::frc971::actions::DrivetrainActionQueueGroup* s);

  virtual void RunAction();
};

// Makes a new DrivetrainAction action.
::std::unique_ptr<::frc971::TypedAction
    < ::frc971::actions::DrivetrainActionQueueGroup>>
MakeDrivetrainAction();

}  // namespace actions
}  // namespace bot3

#endif
