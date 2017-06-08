#ifndef Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2017_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace actors {
using ::frc971::ProfileParameters;

using ::y2017::control_loops::superstructure_queue;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::frc971::autonomous::AutonomousActionQueueGroup *s);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams &params) override;
 private:
  void Reset() {
    intake_goal_ = 0.0;
    turret_goal_ = 0.0;
    vision_track_ = false;
    hood_goal_ = 0.6;
    shooter_velocity_ = 0.0;
    indexer_angular_velocity_ = 0.0;
    intake_max_velocity_ = 0.5;
    gear_servo_ = 0.66;
    use_vision_for_shots_ = false;
    InitializeEncoders();
    ResetDrivetrain();
    SendSuperstructureGoal();
  }

  double intake_goal_ = 0.0;
  double turret_goal_ = 0.0;
  bool vision_track_ = false;
  double hood_goal_ = 0.6;
  double shooter_velocity_ = 0.0;
  double indexer_angular_velocity_ = 0.0;
  double intake_max_velocity_ = 0.5;
  double gear_servo_ = 0.66;
  bool use_vision_for_shots_ = false;

  void set_intake_goal(double intake_goal) { intake_goal_ = intake_goal; }
  void set_turret_goal(double turret_goal) { turret_goal_ = turret_goal; }
  void set_vision_track(bool vision_track) { vision_track_ = vision_track; }
  void set_hood_goal(double hood_goal) { hood_goal_ = hood_goal; }
  void set_shooter_velocity(double shooter_velocity) {
    shooter_velocity_ = shooter_velocity;
  }
  void set_indexer_angular_velocity(double indexer_angular_velocity) {
    indexer_angular_velocity_ = indexer_angular_velocity;
  }
  void set_intake_max_velocity(double intake_max_velocity) {
    intake_max_velocity_ = intake_max_velocity;
  }
  void set_gear_servo(double gear_servo) {
    gear_servo_ = gear_servo;
  }
  void set_use_vision_for_shots(bool use_vision_for_shots) {
    use_vision_for_shots_ = use_vision_for_shots;
  }

  void SendSuperstructureGoal() {
    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
    new_superstructure_goal->intake.distance = intake_goal_;
    new_superstructure_goal->intake.disable_intake = false;
    new_superstructure_goal->turret.angle = turret_goal_;
    new_superstructure_goal->turret.track = vision_track_;
    new_superstructure_goal->hood.angle = hood_goal_;
    new_superstructure_goal->shooter.angular_velocity = shooter_velocity_;

    new_superstructure_goal->intake.profile_params.max_velocity =
        intake_max_velocity_;
    new_superstructure_goal->turret.profile_params.max_velocity = 6.0;
    new_superstructure_goal->hood.profile_params.max_velocity = 5.0;

    new_superstructure_goal->intake.profile_params.max_acceleration = 5.0;
    new_superstructure_goal->turret.profile_params.max_acceleration = 15.0;
    new_superstructure_goal->hood.profile_params.max_acceleration = 25.0;

    new_superstructure_goal->intake.voltage_rollers = 0.0;
    new_superstructure_goal->lights_on = true;
    new_superstructure_goal->intake.disable_intake = false;
    new_superstructure_goal->intake.gear_servo = gear_servo_;
    new_superstructure_goal->use_vision_for_shots = use_vision_for_shots_;

    new_superstructure_goal->indexer.angular_velocity =
        indexer_angular_velocity_;

    if (indexer_angular_velocity_ < -0.1) {
      new_superstructure_goal->indexer.voltage_rollers = 12.0;
      new_superstructure_goal->intake.voltage_rollers = 6.0;
    } else {
      new_superstructure_goal->indexer.voltage_rollers = 0.0;
    }

    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }
};

}  // namespace actors
}  // namespace y2017

#endif  // Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
