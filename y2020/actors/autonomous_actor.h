#ifndef y2020_ACTORS_AUTONOMOUS_ACTOR_H_
#define y2020_ACTORS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2020/actors/auto_splines.h"
#include "y2020/vision/galactic_search_path_generated.h"
#include "y2020/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

namespace y2020 {
namespace actors {

using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

namespace superstructure = y2020::control_loops::superstructure;

class AutonomousActor : public frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(aos::EventLoop *event_loop);

  bool RunAction(
      const frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset();

  void set_intake_goal(double intake_goal) { intake_goal_ = intake_goal; }
  void set_roller_voltage(double roller_voltage) {
    roller_voltage_ = roller_voltage;
  }

  void SendSuperstructureGoal();
  void RetractIntake();
  void SplineAuto();
  void SendStartingPosition(const Eigen::Vector3d &start);
  void GalacticSearch();
  void AutoNavBounce();
  void AutoNavBarrel();
  void AutoNavSlalom();
  bool DriveFwd();

  void Replan();

  double intake_goal_ = 0.0;
  double roller_voltage_ = 0.0;
  const float kRollerSpeedCompensation = 2.0;

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  aos::Sender<y2020::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<y2020::vision::GalacticSearchPath> path_fetcher_;

  aos::TimerHandler *replan_timer_;

  std::optional<std::array<SplineHandle, 4>> bounce_splines_;

  struct GalacticSearchSplines {
    SplineHandle red_a;
    SplineHandle red_b;
    SplineHandle blue_a;
    SplineHandle blue_b;
  };
  std::optional<GalacticSearchSplines> galactic_search_splines_;

  std::optional<SplineHandle> barrel_spline_;
  std::optional<SplineHandle> slalom_spline_;
  std::optional<SplineHandle> test_spline_;

  aos::Alliance alliance_ = aos::Alliance::kInvalid;
  AutonomousSplines auto_splines_;
};

}  // namespace actors
}  // namespace y2020

#endif  // y2020_ACTORS_AUTONOMOUS_ACTOR_H_
