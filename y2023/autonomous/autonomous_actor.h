#ifndef Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
#define Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/user_button_localized_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2023/autonomous/auto_splines.h"
#include "y2023/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2023/control_loops/superstructure/superstructure_status_generated.h"

namespace y2023::autonomous {

class AutonomousActor
    : public ::frc971::autonomous::UserButtonLocalizedAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

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

  bool Run(const ::frc971::autonomous::AutonomousActionParams *params) override;
  void Replan() override;
  void SendStartingPosition(const Eigen::Vector3d &start) override;
  void Reset() override;

  [[nodiscard]] bool WaitForArmGoal(double distance_to_go = 0.01);

  [[nodiscard]] bool WaitForPreloaded();

  void SplineAuto();
  void ChargedUp();
  void ChargedUpCableSide();

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;

  double wrist_goal_;
  control_loops::superstructure::RollerGoal roller_goal_ =
      control_loops::superstructure::RollerGoal::IDLE;

  AutonomousSplines auto_splines_;

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

}  // namespace y2023::autonomous

#endif  // Y2023_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
