#ifndef Y2023_BOT3_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
#define Y2023_BOT3_AUTONOMOUS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2023_bot3/autonomous/auto_splines.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023_bot3/control_loops/superstructure/superstructure_status_generated.h"

// TODO<FILIP>: Add NEUTRAL pivot pose.

namespace y2023_bot3 {
namespace autonomous {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void set_preloaded(bool preloaded) { preloaded_ = preloaded; }

  void set_pivot_goal(
      control_loops::superstructure::PivotGoal requested_pivot_goal) {
    pivot_goal_ = requested_pivot_goal;
  }

  void set_roller_goal(
      control_loops::superstructure::RollerGoal requested_roller_goal) {
    roller_goal_ = requested_roller_goal;
  }

  void SendSuperstructureGoal();

  [[nodiscard]] bool WaitForPreloaded();

  void HighScore();
  void MidScore();
  void LowScore();
  void Spit();
  void SpitHigh();
  void StopSpitting();
  void Pickup();
  void Intake();
  void Neutral();

  void Reset();

  void SendStartingPosition(const Eigen::Vector3d &start);
  void MaybeSendStartingPosition();
  void Replan();
  void ChargedUp();
  void ChargedUpMiddle();
  void OnePieceMiddle();

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<aos::RobotState> robot_state_fetcher_;

  aos::TimerHandler *replan_timer_;
  aos::TimerHandler *button_poll_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  AutonomousSplines auto_splines_;
  bool user_indicated_safe_to_reset_ = false;
  bool sent_starting_position_ = false;

  bool is_planned_ = false;

  std::optional<Eigen::Vector3d> starting_position_;

  bool preloaded_ = false;

  control_loops::superstructure::PivotGoal pivot_goal_;
  control_loops::superstructure::RollerGoal roller_goal_;

  aos::Sender<control_loops::superstructure::Goal> superstructure_goal_sender_;
  aos::Fetcher<y2023_bot3::control_loops::superstructure::Status>
      superstructure_status_fetcher_;

  std::optional<SplineHandle> test_spline_;
  std::optional<std::array<SplineHandle, 4>> charged_up_splines_;
  std::optional<std::array<SplineHandle, 1>> charged_up_middle_splines_;
};

}  // namespace autonomous
}  // namespace y2023_bot3

#endif  // Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
