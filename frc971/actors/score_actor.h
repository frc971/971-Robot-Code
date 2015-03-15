#ifndef FRC971_ACTORS_SCORE_ACTOR_H_
#define FRC971_ACTORS_SCORE_ACTOR_H_

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "aos/common/util/kinematics.h"
#include "frc971/actors/score_action.q.h"

namespace frc971 {
namespace actors {

class ScoreActor
    : public ::aos::common::actions::ActorBase<ScoreActionQueueGroup> {
 public:
  explicit ScoreActor(ScoreActionQueueGroup *queues);

  bool RunAction(const ScoreParams &params) override;

 private:

  ::aos::util::ElevatorArmKinematics kinematics_;
  bool NearGoal(double x, double y);
  bool PlaceTheStack(const ScoreParams &params);
  bool MoveStackIntoPosition(const ScoreParams &params);
  bool SendGoal(double x, double y, bool grabbers_enabled);
  double CurrentHeight();
  double CurrentX();
};

typedef aos::common::actions::TypedAction<ScoreActionQueueGroup> ScoreAction;

// Makes a new ScoreActor action.
::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams &params);

}  // namespace actors
}  // namespace frc971

#endif
