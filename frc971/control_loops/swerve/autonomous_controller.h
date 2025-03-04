#ifndef FRC971_CONTROL_LOOPS_SWERVE
#define FRC971_CONTROL_LOOPS_SWERVE

#include "aos/events/event_loop.h"
#include "frc971/control_loops/swerve/autonomous_controller_debug_static.h"
#include "frc971/control_loops/swerve/autonomous_init_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/control_loops/swerve/swerve_localizer_state_generated.h"
#include "frc971/control_loops/swerve/swerve_trajectory_static.h"
#include "frc971/input/joystick_state_generated.h"

ABSL_DECLARE_FLAG(double, kVxProportionalGain);
ABSL_DECLARE_FLAG(double, kVyProportionalGain);
ABSL_DECLARE_FLAG(double, kOmegaProportionalGain);
ABSL_DECLARE_FLAG(bool, kUseEkfState);

namespace frc971::control_loops::swerve {

class AutonomousController {
 public:
  AutonomousController(
      aos::EventLoop *event_loop, std::string_view trajectory_path,
      const std::unordered_map<std::string_view, std::function<void()>>
          &callbacks);
  void Iterate();

  bool Completed();

  void AddCallback(std::function<void()> callback,
                   std::chrono::milliseconds delay);

 private:
  // nullopt when we aren't running, when we're running its the index of the
  // running trajectory
  std::optional<size_t> trajectory_index_;

  aos::FlatbufferVector<frc971::control_loops::swerve::SwerveTrajectory>
      trajectory_;

  aos::Sender<GoalStatic> swerve_goal_sender_;
  aos::Sender<AutonomousInitStatic> autonomous_init_sender_;
  aos::Sender<AutonomousControllerDebugStatic>
      autonomous_controller_debug_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;
  aos::Fetcher<frc971::control_loops::swerve::LocalizerState>
      localizer_state_fetcher_;

  struct Action {
    bool completed;
    double time;
    std::shared_ptr<std::function<void()>> callback;
  };

  std::vector<Action> actions_;

  bool flipped_;

  bool completed_;

  double prev_x_;
  double prev_y_;
  double prev_theta_;

  aos::EventLoop *event_loop_;
};

}  // namespace frc971::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE
