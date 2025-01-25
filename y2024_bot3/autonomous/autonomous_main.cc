#include <filesystem>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/swerve/autonomous_controller.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_goal_static.h"

namespace y2024_bot3::autonomous {

using y2024_bot3::control_loops::superstructure::GoalStatic;
using y2024_bot3::control_loops::superstructure::IntakeGoal;
using y2024_bot3::control_loops::superstructure::PivotGoal;

class AutoGoalSender {
 public:
  AutoGoalSender(::aos::EventLoop *event_loop)
      : autonomous_controller_(nullptr) {
    std::unordered_map<std::string_view, std::function<void()>> callbacks;

    autonomous_controller_ =
        std::make_unique<frc971::control_loops::swerve::AutonomousController>(
            event_loop, "trajectory.bfbs", callbacks);
  }

 private:
  std::unique_ptr<frc971::control_loops::swerve::AutonomousController>
      autonomous_controller_;
};

}  // namespace y2024_bot3::autonomous

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024_bot3::autonomous::AutoGoalSender auto_goal_sender(&event_loop);

  event_loop.Run();

  return 0;
}
