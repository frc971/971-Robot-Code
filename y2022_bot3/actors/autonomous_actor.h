#ifndef Y2022_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2022_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2022_bot3/actors/auto_splines.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022_bot3 {
namespace actors {

using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

namespace superstructure = y2022_bot3::control_loops::superstructure;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset();

  void SendSuperstructureGoal();
  void SendStartingPosition(const Eigen::Vector3d &start);
  void MaybeSendStartingPosition();

  void SplineAuto();

  void Replan();

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Sender<y2022_bot3::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  aos::Fetcher<y2022_bot3::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<aos::RobotState> robot_state_fetcher_;

  aos::TimerHandler *replan_timer_;
  aos::TimerHandler *button_poll_;

  std::optional<SplineHandle> test_spline_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  AutonomousSplines auto_splines_;
  bool user_indicated_safe_to_reset_ = false;
  bool sent_starting_position_ = false;

  bool is_planned_ = false;

  std::optional<Eigen::Vector3d> starting_position_;
};

}  // namespace actors
}  // namespace y2022_bot3

#endif  // Y2022_BOT3_ACTORS_AUTONOMOUS_ACTOR_H_
