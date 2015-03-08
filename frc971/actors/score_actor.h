#ifndef FRC971_ACTORS_SCORE_ACTOR_H_
#define FRC971_ACTORS_SCORE_ACTOR_H_

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/score_action.q.h"
#include "frc971/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {

class ScoreActor : public FridgeActorBase<ScoreActionQueueGroup> {
 public:
  explicit ScoreActor(ScoreActionQueueGroup *queues);

  bool RunAction(const ScoreParams &params) override;

 private:
  // Creates and runs a profile action for the fridge. Handles cancelling
  // correctly.
  // height: How high we want the fridge to go.
  // angle: What angle we want the arm to be at.
  // grabbers: Whether we want the grabbers deployed or not.
  void DoProfile(double height, double angle, bool grabbers);
};

typedef aos::common::actions::TypedAction<ScoreActionQueueGroup> ScoreAction;

// Makes a new ScoreActor action.
::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams &params);

}  // namespace actors
}  // namespace frc971

#endif
