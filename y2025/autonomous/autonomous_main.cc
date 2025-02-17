#include <filesystem>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/swerve/autonomous_controller.h"
#include "y2025/autonomous/autonomous_superstructure_goal_static.h"
#include "y2025/control_loops/superstructure/superstructure_goal_generated.h"

namespace y2025::autonomous {

using y2025::control_loops::superstructure::ClimberGoal;
using y2025::control_loops::superstructure::ElevatorGoal;
using y2025::control_loops::superstructure::EndEffectorGoal;
using y2025::control_loops::superstructure::GoalStatic;
using y2025::control_loops::superstructure::PivotGoal;
using y2025::control_loops::superstructure::RobotSide;
using y2025::control_loops::superstructure::WristGoal;

class AutoGoalSender {
 public:
  AutoGoalSender(::aos::EventLoop *event_loop)
      : autonomous_controller_(nullptr),
        autonomous_superstructure_goal_sender_(
            event_loop->MakeSender<
                y2025::autonomous::AutonomousSuperstructureGoalStatic>(
                "/autonomous")) {
    std::unordered_map<std::string_view, std::function<void()>> callbacks;

    callbacks.emplace("elevator up", [this]() {
      SendAutonomousSuperstructureGoal(
          EndEffectorGoal::INTAKE, ElevatorGoal::SCORE_L3, PivotGoal::SCORE_L3,
          std::nullopt, WristGoal::SCORE_L3, RobotSide::FRONT, std::nullopt);
    });
    callbacks.emplace("elevator down", [this]() {
      SendAutonomousSuperstructureGoal(
          EndEffectorGoal::NEUTRAL, ElevatorGoal::NEUTRAL, PivotGoal::NEUTRAL,
          std::nullopt, WristGoal::NEUTRAL, std::nullopt, std::nullopt);
    });

    autonomous_controller_ =
        std::make_unique<frc971::control_loops::swerve::AutonomousController>(
            event_loop, "trajectory.bfbs", callbacks);
  }

  void SendAutonomousSuperstructureGoal(
      std::optional<EndEffectorGoal> end_effector_goal,
      std::optional<ElevatorGoal> elevator_goal,
      std::optional<PivotGoal> pivot_goal,
      std::optional<ClimberGoal> climber_goal,
      std::optional<WristGoal> wrist_goal, std::optional<RobotSide> robot_side,
      std::optional<aos::monotonic_clock::duration> duration) {
    aos::Sender<y2025::autonomous::AutonomousSuperstructureGoalStatic>::
        StaticBuilder autonomous_superstructure_goal_builder =
            autonomous_superstructure_goal_sender_.MakeStaticBuilder();

    GoalStatic *goal = autonomous_superstructure_goal_builder->add_goal();
    y2025::autonomous::GoalsSetStatic *goals_set =
        autonomous_superstructure_goal_builder->add_goals_set();

    if (end_effector_goal) {
      goal->set_end_effector_goal(end_effector_goal.value());
    }
    goals_set->set_end_effector(end_effector_goal.has_value());
    if (pivot_goal) {
      goal->set_pivot_goal(pivot_goal.value());
    }
    goals_set->set_pivot(pivot_goal.has_value());
    if (elevator_goal) {
      goal->set_elevator_goal(elevator_goal.value());
    }
    goals_set->set_elevator(elevator_goal.has_value());
    if (climber_goal) {
      goal->set_climber_goal(climber_goal.value());
    }
    goals_set->set_climber(climber_goal.has_value());
    if (wrist_goal) {
      goal->set_wrist_goal(wrist_goal.value());
    }
    goals_set->set_wrist(wrist_goal.has_value());
    if (robot_side) {
      goal->set_robot_side(robot_side.value());
    }
    goals_set->set_robot_side(robot_side.has_value());

    autonomous_superstructure_goal_builder->set_timed(duration.has_value());
    autonomous_superstructure_goal_builder->set_duration(
        duration.value_or(std::chrono::nanoseconds(0)).count());
    autonomous_superstructure_goal_builder.CheckOk(
        autonomous_superstructure_goal_builder.Send());
  }

 private:
  std::unique_ptr<frc971::control_loops::swerve::AutonomousController>
      autonomous_controller_;
  aos::Sender<y2025::autonomous::AutonomousSuperstructureGoalStatic>
      autonomous_superstructure_goal_sender_;
};

}  // namespace y2025::autonomous

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2025::autonomous::AutoGoalSender auto_goal_sender(&event_loop);

  event_loop.Run();

  return 0;
}
