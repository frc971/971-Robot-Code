#ifndef Y2014_ACTORS_SHOOT_ACTOR_H_
#define Y2014_ACTORS_SHOOT_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"

#include "y2014/actors/shoot_action.q.h"

namespace y2014 {
namespace actors {

class ShootActor
    : public ::aos::common::actions::ActorBase<actors::ShootActionQueueGroup> {
 public:
  explicit ShootActor(actors::ShootActionQueueGroup* s);

  // Actually execute the action of moving the claw and shooter into position
  // and actually firing them.
  bool RunAction(const double &params) override;
  void InnerRunAction();

  // calc an offset to our requested shot based on robot speed
  double SpeedToAngleOffset(double speed);

  static constexpr double kOffsetRadians = 0.4;
  static constexpr double kClawShootingSeparation = 0.10;
  static constexpr double kClawShootingSeparationGoal = 0.10;

 protected:
  // completed shot
  bool DoneShot();
  // ready for shot
  bool DonePreShotOpen();
  // in the right place
  bool DoneSetupShot();

  // to track when shot is complete
  int previous_shots_;
};

typedef ::aos::common::actions::TypedAction<actors::ShootActionQueueGroup>
    ShootAction;

// Makes a new ShootActor action.
::std::unique_ptr<ShootAction> MakeShootAction();

}  // namespace actors
}  // namespace y2014

#endif  // Y2014_ACTORS_SHOOT_ACTOR_H_
