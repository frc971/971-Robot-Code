#ifndef Y2015_ACTORS_SCORE_ACTOR_H_
#define Y2015_ACTORS_SCORE_ACTOR_H_

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/util/kinematics.h"
#include "y2015/actors/score_action.q.h"

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
  bool NearHorizontalGoal(double x);
  bool PlaceTheStack(const ScoreParams &params);
  bool MoveStackIntoPosition(const ScoreParams &params);
  bool SendGoal(double x, double y, bool grabbers_enabled,
                double max_x_velocity, double max_y_velocity,
                double max_x_acceleration, double max_y_acceleration);
  double CurrentHeight();
  double CurrentGoalHeight();
  double CurrentX();
  double CurrentGoalX();
};

typedef aos::common::actions::TypedAction<ScoreActionQueueGroup> ScoreAction;

// Makes a new ScoreActor action.
::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams &params);

}  // namespace actors
}  // namespace frc971

#endif
