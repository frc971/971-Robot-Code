#ifndef FRC971_ACTIONS_CLAW_ACTION_H_
#define FRC971_ACTIONS_CLAW_ACTION_H_

#include <memory>

#include "frc971/actors/claw_action.q.h"
#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "aos/common/util/trapezoid_profile.h"

namespace frc971 {
namespace actors {
namespace testing {
class ClawActorTest_ValidGoals_Test;
}

class ClawActor : public aos::common::actions::ActorBase<ClawActionQueueGroup> {
 public:
  explicit ClawActor(ClawActionQueueGroup *s);
  bool RunAction(const ClawParams &params) override;

 private:
  friend class testing::ClawActorTest_ValidGoals_Test;

  // Returns true if it's reached its ultimate goal, false otherwise.
  bool Iterate(const ClawParams &params);

  ::aos::util::TrapezoidProfile profile_;
};

typedef aos::common::actions::TypedAction<ClawActionQueueGroup> ClawAction;

// Makes a new FridgeProfileActor action.
::std::unique_ptr<ClawAction> MakeClawAction(const ClawParams &claw_params);

}  // namespace actors
}  // namespace frc971

#endif
