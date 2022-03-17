#ifndef Y2022_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2022_ACTORS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2022/actors/auto_splines.h"
#include "y2022/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"

namespace y2022 {
namespace actors {

using control_loops::superstructure::RequestedIntake;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

namespace superstructure = y2022::control_loops::superstructure;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset();

  void set_intake_front_goal(double intake_front_goal) {
    intake_front_goal_ = intake_front_goal;
  }
  void set_intake_back_goal(double intake_back_goal) {
    intake_back_goal_ = intake_back_goal;
  }
  void set_roller_front_voltage(double roller_front_voltage) {
    roller_front_voltage_ = roller_front_voltage;
  }
  void set_roller_back_voltage(double roller_back_voltage) {
    roller_back_voltage_ = roller_back_voltage;
  }
  void set_transfer_roller_front_voltage(double voltage) {
    transfer_roller_front_voltage_ = voltage;
  }
  void set_transfer_roller_back_voltage(double voltage) {
    transfer_roller_back_voltage_ = voltage;
  }
  void set_requested_intake(std::optional<RequestedIntake> requested_intake) {
    requested_intake_ = requested_intake;
  }
  void set_turret_goal(double turret_goal) {
    turret_goal_ = turret_goal;
  }

  void set_fire_at_will(bool fire) { fire_ = fire; }
  void set_preloaded(bool preloaded) { preloaded_ = preloaded; }

  void SendSuperstructureGoal();
  void ExtendFrontIntake();
  void RetractFrontIntake();
  void ExtendBackIntake();
  void RetractBackIntake();
  void SendStartingPosition(const Eigen::Vector3d &start);
  void MaybeSendStartingPosition();

  // Tells the superstructure the ball was preloaded and waits until it updates
  // the state
  [[nodiscard]] bool WaitForPreloaded();
  // Waits for a certain number of balls to be shot
  [[nodiscard]] bool WaitForBallsShot(int num_shot);

  void SplineAuto();
  void RapidReact();

  void Replan();

  double intake_front_goal_ = 0.0;
  double intake_back_goal_ = 0.0;
  double roller_front_voltage_ = 0.0;
  double roller_back_voltage_ = 0.0;
  double transfer_roller_front_voltage_ = 0.0;
  double transfer_roller_back_voltage_ = 0.0;
  std::optional<RequestedIntake> requested_intake_ = std::nullopt;
  double turret_goal_ = 0.0;
  bool fire_ = false;
  bool preloaded_ = false;

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Sender<y2022::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  aos::Fetcher<y2022::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<aos::RobotState> robot_state_fetcher_;

  aos::TimerHandler *replan_timer_;
  aos::TimerHandler *button_poll_;

  std::optional<SplineHandle> test_spline_;
  std::optional<std::array<SplineHandle, 3>> rapid_react_splines_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  AutonomousSplines auto_splines_;
  bool user_indicated_safe_to_reset_ = false;
  bool sent_starting_position_ = false;

  bool is_planned_ = false;

  std::optional<Eigen::Vector3d> starting_position_;
};

}  // namespace actors
}  // namespace y2022

#endif  // Y2022_ACTORS_AUTONOMOUS_ACTOR_H_
