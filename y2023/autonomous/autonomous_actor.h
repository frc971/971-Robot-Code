#ifndef Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
#define Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2023/autonomous/auto_splines.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023 {
namespace autonomous {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void set_arm_goal_position(uint32_t requested_arm_goal_position) {
    arm_goal_position_ = requested_arm_goal_position;
  }

  void set_roller_goal(
      control_loops::superstructure::RollerGoal requested_roller_goal) {
    roller_goal_ = requested_roller_goal;
  }

  void set_wrist_goal(double requested_wrist_goal) {
    wrist_goal_ = requested_wrist_goal;
  }

  void set_preloaded(bool preloaded) { preloaded_ = preloaded; }

  void SendSuperstructureGoal();
  void HighCubeScore();
  void MidCubeScore();
  void MidConeScore();
  void Neutral();
  void PickupCube();
  void Spit();
  void StopSpitting();
  void IntakeCube();
  void Balance();

  [[nodiscard]] bool WaitForArmGoal(double distance_to_go = 0.01);

  [[nodiscard]] bool WaitForPreloaded();

  void Reset();

  void SendStartingPosition(const Eigen::Vector3d &start);
  void MaybeSendStartingPosition();
  void SplineAuto();
  void ChargedUp();
  void ChargedUpCableSide();
  void Replan();

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<aos::RobotState> robot_state_fetcher_;

  double wrist_goal_;
  control_loops::superstructure::RollerGoal roller_goal_ =
      control_loops::superstructure::RollerGoal::IDLE;

  aos::TimerHandler *replan_timer_;
  aos::TimerHandler *button_poll_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  AutonomousSplines auto_splines_;
  bool user_indicated_safe_to_reset_ = false;
  bool sent_starting_position_ = false;

  bool is_planned_ = false;

  std::optional<Eigen::Vector3d> starting_position_;

  uint32_t arm_goal_position_;
  bool preloaded_ = false;

  aos::Sender<control_loops::superstructure::Goal> superstructure_goal_sender_;
  aos::Fetcher<y2023::control_loops::superstructure::Status>
      superstructure_status_fetcher_;

  std::optional<SplineHandle> test_spline_;
  std::optional<std::array<SplineHandle, 4>> charged_up_splines_;
  std::optional<std::array<SplineHandle, 4>> charged_up_cable_splines_;

  // List of arm angles from arm::PointsList
  const ::std::vector<::Eigen::Matrix<double, 3, 1>> points_;
};

}  // namespace autonomous
}  // namespace y2023

#endif  // Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
