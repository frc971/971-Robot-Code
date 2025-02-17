#include <filesystem>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2025/autonomous/autonomous_superstructure_goal_generated.h"
#include "y2025/control_loops/superstructure/superstructure_goal_static.h"

namespace y2025::autonomous {

using y2025::control_loops::superstructure::ClimberGoal;
using y2025::control_loops::superstructure::ElevatorGoal;
using y2025::control_loops::superstructure::EndEffectorGoal;
using y2025::control_loops::superstructure::PivotGoal;
using y2025::control_loops::superstructure::RobotSide;
using y2025::control_loops::superstructure::WristGoal;

template <typename GoalType>
class TimedGoal {
 public:
  TimedGoal(GoalType default_goal)
      : goal_(default_goal), default_goal_(default_goal) {}

  void Update(aos::monotonic_clock::duration dt) {
    if (time_left_) {
      time_left_.value() -= dt;
      if (time_left_.value().count() <= 0) {
        time_left_ = std::nullopt;
        goal_ = default_goal_;
      }
    }
  }

  GoalType goal() { return goal_; }

  void setGoal(GoalType goal) { goal_ = goal; }

  void setTime(aos::monotonic_clock::duration time) { time_left_ = time; }

 private:
  GoalType goal_;
  const GoalType default_goal_;
  std::optional<aos::monotonic_clock::duration> time_left_;
};

class AutoSuperstructure {
 public:
  AutoSuperstructure(::aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        superstructure_goal_sender_(
            event_loop
                ->MakeSender<y2025::control_loops::superstructure::GoalStatic>(
                    "/autonomous")),
        end_effector_(EndEffectorGoal::NEUTRAL),
        elevator_(ElevatorGoal::NEUTRAL),
        pivot_(PivotGoal::NEUTRAL),
        climber_(ClimberGoal::NEUTRAL),
        wrist_(WristGoal::NEUTRAL),
        robot_side_(RobotSide::FRONT) {
    event_loop_->MakeWatcher(
        "/autonomous",
        [this](const y2025::autonomous::AutonomousSuperstructureGoal
                   &autonomous_superstructure_goal) {
          HandleAutonomousSuperstructureGoal(autonomous_superstructure_goal);
        });

    const auto timer = event_loop->AddTimer([this]() {
      end_effector_.Update(std::chrono::milliseconds(5));
      elevator_.Update(std::chrono::milliseconds(5));
      pivot_.Update(std::chrono::milliseconds(5));
      climber_.Update(std::chrono::milliseconds(5));
      wrist_.Update(std::chrono::milliseconds(5));

      aos::Sender<y2025::control_loops::superstructure::GoalStatic>::
          StaticBuilder superstructure_goal_builder =
              superstructure_goal_sender_.MakeStaticBuilder();

      superstructure_goal_builder->set_end_effector_goal(end_effector_.goal());
      superstructure_goal_builder->set_elevator_goal(elevator_.goal());
      superstructure_goal_builder->set_pivot_goal(pivot_.goal());
      superstructure_goal_builder->set_climber_goal(climber_.goal());
      superstructure_goal_builder->set_wrist_goal(wrist_.goal());
      superstructure_goal_builder->set_robot_side(robot_side_);

      superstructure_goal_builder.CheckOk(superstructure_goal_builder.Send());
    });

    event_loop_->OnRun([timer, event_loop]() {
      timer->Schedule(event_loop->monotonic_now(),
                      std::chrono::milliseconds(5));
    });
  }

 private:
  aos::EventLoop *event_loop_;

  aos::Sender<y2025::control_loops::superstructure::GoalStatic>
      superstructure_goal_sender_;

  TimedGoal<EndEffectorGoal> end_effector_;

  TimedGoal<ElevatorGoal> elevator_;

  TimedGoal<PivotGoal> pivot_;

  TimedGoal<ClimberGoal> climber_;

  TimedGoal<WristGoal> wrist_;

  RobotSide robot_side_;

  void HandleAutonomousSuperstructureGoal(
      const y2025::autonomous::AutonomousSuperstructureGoal
          &autonomous_superstructure_goal) {
    bool timed = autonomous_superstructure_goal.timed();
    aos::monotonic_clock::duration duration =
        std::chrono::nanoseconds(autonomous_superstructure_goal.duration());

    if (autonomous_superstructure_goal.goals_set()->end_effector()) {
      end_effector_.setGoal(
          autonomous_superstructure_goal.goal()->end_effector_goal());
      if (timed) {
        end_effector_.setTime(duration);
      }
    }

    if (autonomous_superstructure_goal.goals_set()->pivot()) {
      pivot_.setGoal(autonomous_superstructure_goal.goal()->pivot_goal());
      if (timed) {
        pivot_.setTime(duration);
      }
    }

    if (autonomous_superstructure_goal.goals_set()->elevator()) {
      elevator_.setGoal(autonomous_superstructure_goal.goal()->elevator_goal());
      if (timed) {
        elevator_.setTime(duration);
      }
    }

    if (autonomous_superstructure_goal.goals_set()->climber()) {
      climber_.setGoal(autonomous_superstructure_goal.goal()->climber_goal());
      if (timed) {
        climber_.setTime(duration);
      }
    }

    if (autonomous_superstructure_goal.goals_set()->wrist()) {
      wrist_.setGoal(autonomous_superstructure_goal.goal()->wrist_goal());
      if (timed) {
        wrist_.setTime(duration);
      }
    }

    if (autonomous_superstructure_goal.goals_set()->robot_side()) {
      robot_side_ = autonomous_superstructure_goal.goal()->robot_side();
    }
  }
};

}  // namespace y2025::autonomous

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2025::autonomous::AutoSuperstructure auto_goal_sender(&event_loop);

  event_loop.Run();

  return 0;
}
