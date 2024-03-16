#ifndef Y2024_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
#define Y2024_AUTONOMOUS_AUTONOMOUS_ACTOR_H_

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/user_button_localized_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2024/autonomous/auto_splines.h"
#include "y2024/constants.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/superstructure/superstructure_goal_static.h"
#include "y2024/control_loops/superstructure/superstructure_status_static.h"

namespace y2024::autonomous {

class AutonomousActor
    : public ::frc971::autonomous::UserButtonLocalizedAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop,
                           const y2024::Constants *robot_constants);

 private:
  void set_intake_goal(control_loops::superstructure::IntakeGoal intake_goal) {
    intake_goal_ = intake_goal;
  }
  void set_note_goal(control_loops::superstructure::NoteGoal note_goal) {
    note_goal_ = note_goal;
  }
  void set_auto_aim(bool auto_aim) { auto_aim_ = auto_aim; }
  void set_fire(bool fire) { fire_ = fire; }
  void set_preloaded(bool preloaded) { preloaded_ = preloaded; }

  bool Run(const ::frc971::autonomous::AutonomousActionParams *params) override;
  void Replan() override;
  void SendStartingPosition(const Eigen::Vector3d &start) override;
  void Reset() override;

  void SplineAuto();
  void MobilityAndShoot();
  void FourPieceAuto();
  void SendSuperstructureGoal();

  void Intake();
  void Aim();
  void Shoot();
  void StopFiring();

  uint32_t shot_count();

  [[nodiscard]] bool WaitForPreloaded();
  [[nodiscard]] bool WaitForNoteFired(uint32_t penultimate_target_shot_count,
                                      std::chrono::nanoseconds timeout);
  [[nodiscard]] bool WaitForCatapultReady();

  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;

  aos::Sender<control_loops::superstructure::GoalStatic>
      superstructure_goal_sender_;

  aos::Fetcher<y2024::control_loops::superstructure::Status>
      superstructure_status_fetcher_;

  const Constants *robot_constants_;

  AutonomousSplines auto_splines_;

  std::optional<SplineHandle> test_spline_;
  std::optional<std::array<SplineHandle, 1>> mobility_and_shoot_splines_;
  std::optional<std::array<SplineHandle, 5>> four_piece_splines_;

  control_loops::superstructure::IntakeGoal intake_goal_ =
      control_loops::superstructure::IntakeGoal::NONE;

  control_loops::superstructure::NoteGoal note_goal_ =
      control_loops::superstructure::NoteGoal::CATAPULT;

  bool auto_aim_ = false;
  bool fire_ = false;
  bool preloaded_ = false;
};

}  // namespace y2024::autonomous

#endif  // Y2024_AUTONOMOUS_AUTONOMOUS_ACTOR_H_
