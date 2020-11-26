#ifndef y2020_ACTORS_AUTONOMOUS_ACTOR_H_
#define y2020_ACTORS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"

namespace y2020 {
namespace actors {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset();
  void SplineAuto();
  bool DriveFwd();

  ::aos::Sender<::frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Alliance alliance_ = aos::Alliance::kInvalid;
};

}  // namespace actors
}  // namespace y2020

#endif  // y2020_ACTORS_AUTONOMOUS_ACTOR_H_
