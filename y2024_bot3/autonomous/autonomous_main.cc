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
      : superstructure_goal_sender_(
            event_loop->MakeSender<GoalStatic>("/superstructure")),
        autonomous_controller_(nullptr) {
    std::unordered_map<std::string_view, std::function<void()>> callbacks;
    callbacks.emplace(std::make_pair("Intake", [this]() {
      set_goals(PivotGoal::INTAKE, IntakeGoal::INTAKE);
    }));
    callbacks.emplace(std::make_pair("Outtake", [this]() {
      set_goals(PivotGoal::INTAKE, IntakeGoal::SPIT);
    }));

    autonomous_controller_ =
        std::make_unique<frc971::control_loops::swerve::AutonomousController>(
            event_loop, "trajectory/trajectory.json", callbacks);
  }

  void set_goals(PivotGoal pivot_goal, IntakeGoal intake_goal) {
    auto goal_builder = this->superstructure_goal_sender_.MakeStaticBuilder();

    goal_builder->set_arm_position(pivot_goal);
    goal_builder->set_roller_goal(intake_goal);

    goal_builder.CheckOk(goal_builder.Send());
  }

 private:
  aos::Sender<GoalStatic> superstructure_goal_sender_;
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
