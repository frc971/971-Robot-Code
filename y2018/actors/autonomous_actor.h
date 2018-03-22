#ifndef Y2018_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2018_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"

namespace y2018 {
namespace actors {
using ::y2018::control_loops::superstructure_queue;

namespace arm = ::y2018::control_loops::superstructure::arm;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::frc971::autonomous::AutonomousActionQueueGroup *s);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams &params) override;
 private:
  void Reset() {
    roller_voltage_ = 0.0;
    left_intake_angle_ = -3.3;
    right_intake_angle_ = -3.3;
    arm_goal_position_ = arm::NeutralIndex();
    grab_box_ = false;
    open_claw_ = false;
    deploy_fork_ = false;
    InitializeEncoders();
    ResetDrivetrain();
    SendSuperstructureGoal();
  }

  double roller_voltage_ = 0.0;
  double left_intake_angle_ = -3.3;
  double right_intake_angle_ = -3.3;
  uint32_t arm_goal_position_ = arm::NeutralIndex();
  bool grab_box_ = false;
  bool open_claw_ = false;
  bool deploy_fork_ = false;

  void set_roller_voltage(double roller_voltage) {
    roller_voltage_ = roller_voltage;
  }
  void set_left_intake_angle(double left_intake_angle) {
    left_intake_angle_ = left_intake_angle;
  }
  void set_right_intake_angle(double right_intake_angle) {
    right_intake_angle_ = right_intake_angle;
  }
  void set_arm_goal_position(uint32_t arm_goal_position) {
    arm_goal_position_ = arm_goal_position;
  }
  void set_grab_box(bool grab_box) { grab_box_ = grab_box; }
  void set_open_claw(bool open_claw) { open_claw_ = open_claw; }
  void set_deploy_fork(bool deploy_fork) { deploy_fork_ = deploy_fork; }

  void SendSuperstructureGoal() {
    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
    new_superstructure_goal->intake.roller_voltage = roller_voltage_;
    new_superstructure_goal->intake.left_intake_angle = left_intake_angle_;
    new_superstructure_goal->intake.right_intake_angle = right_intake_angle_;

    new_superstructure_goal->arm_goal_position = arm_goal_position_;
    new_superstructure_goal->grab_box = grab_box_;
    new_superstructure_goal->open_claw = open_claw_;
    new_superstructure_goal->deploy_fork = deploy_fork_;

    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

  bool WaitForArmTrajectoryClose(double threshold) {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        ::std::chrono::milliseconds(5) / 2);
    while (true) {
      if (ShouldCancel()) {
        return false;
      }

      superstructure_queue.status.FetchLatest();
      if (superstructure_queue.status.get()) {
        if (superstructure_queue.status->arm.current_node == arm_goal_position_ &&
            superstructure_queue.status->arm.path_distance_to_go < threshold) {
          return true;
        }
      }
      phased_loop.SleepUntilNext();
    }
  }
};

}  // namespace actors
}  // namespace y2018

#endif  // Y2018_ACTORS_AUTONOMOUS_ACTOR_H_
