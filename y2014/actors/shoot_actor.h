#ifndef Y2014_ACTORS_SHOOT_ACTOR_H_
#define Y2014_ACTORS_SHOOT_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "y2014/control_loops/claw/claw_goal_generated.h"
#include "y2014/control_loops/claw/claw_status_generated.h"
#include "y2014/control_loops/shooter/shooter_goal_generated.h"
#include "y2014/control_loops/shooter/shooter_status_generated.h"

namespace y2014 {
namespace actors {

class ShootActor
    : public ::aos::common::actions::ActorBase<aos::common::actions::Goal> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<
      aos::common::actions::Goal>
      Factory;

  explicit ShootActor(::aos::EventLoop *event_loop);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/shoot_action");
  }

  // Actually execute the action of moving the claw and shooter into position
  // and actually firing them.
  bool RunAction(const aos::common::actions::DoubleParam *params) override;
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
  bool ShooterIsReady();

  bool IntakeOff();
  bool ClawIsReady();

 private:
  ::aos::Fetcher<::y2014::control_loops::claw::Goal> claw_goal_fetcher_;
  ::aos::Fetcher<::y2014::control_loops::claw::Status> claw_status_fetcher_;
  ::aos::Sender<::y2014::control_loops::claw::Goal> claw_goal_sender_;
  ::aos::Fetcher<::y2014::control_loops::shooter::Status>
      shooter_status_fetcher_;
  ::aos::Fetcher<::y2014::control_loops::shooter::Goal> shooter_goal_fetcher_;
  ::aos::Sender<::y2014::control_loops::shooter::Goal> shooter_goal_sender_;

  // to track when shot is complete
  int previous_shots_;
};

}  // namespace actors
}  // namespace y2014

#endif  // Y2014_ACTORS_SHOOT_ACTOR_H_
