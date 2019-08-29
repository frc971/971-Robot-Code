#ifndef Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2017_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2017/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2017/control_loops/superstructure/superstructure_status_generated.h"

namespace y2017 {
namespace actors {
using ::frc971::ProfileParameters;

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

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

  ::aos::Fetcher<::y2017::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  ::aos::Sender<::y2017::control_loops::superstructure::Goal>
      superstructure_goal_sender_;

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

  void WaitForHoodZeroed() {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);
    while (true) {
      if (ShouldCancel()) return;

      superstructure_status_fetcher_.Fetch();
      if (superstructure_status_fetcher_.get()) {
        if (superstructure_status_fetcher_->hood()->zeroed()) {
          return;
        }
      }
      phased_loop.SleepUntilNext();
    }
  }

  void SendSuperstructureGoal() {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<frc971::ProfileParameters>
        intake_profile_parameters_offset = frc971::CreateProfileParameters(
            *builder.fbb(), intake_max_velocity_, 5.0);

    control_loops::superstructure::IntakeGoal::Builder intake_builder =
        builder.MakeBuilder<control_loops::superstructure::IntakeGoal>();
    intake_builder.add_distance(intake_goal_);
    intake_builder.add_disable_intake(false);
    intake_builder.add_profile_params(intake_profile_parameters_offset);
    intake_builder.add_voltage_rollers(0.0);
    intake_builder.add_disable_intake(false);
    intake_builder.add_gear_servo(gear_servo_);
    flatbuffers::Offset<control_loops::superstructure::IntakeGoal>
        intake_offset = intake_builder.Finish();

    control_loops::superstructure::IndexerGoal::Builder indexer_builder =
        builder.MakeBuilder<control_loops::superstructure::IndexerGoal>();
    indexer_builder.add_angular_velocity(indexer_angular_velocity_);
    indexer_builder.add_voltage_rollers(0.0);
    flatbuffers::Offset<control_loops::superstructure::IndexerGoal>
        indexer_offset = indexer_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        turret_profile_parameters_offset = frc971::CreateProfileParameters(
            *builder.fbb(), 6.0, 15.0);
    control_loops::superstructure::TurretGoal::Builder turret_builder =
        builder.MakeBuilder<control_loops::superstructure::TurretGoal>();
    turret_builder.add_angle(turret_goal_);
    turret_builder.add_track(vision_track_);
    turret_builder.add_profile_params(turret_profile_parameters_offset);
    flatbuffers::Offset<control_loops::superstructure::TurretGoal> turret_offset =
      turret_builder.Finish();

    flatbuffers::Offset<frc971::ProfileParameters>
        hood_profile_parameters_offset = frc971::CreateProfileParameters(
            *builder.fbb(), 5.0, 25.0);
    control_loops::superstructure::HoodGoal::Builder hood_builder =
        builder.MakeBuilder<control_loops::superstructure::HoodGoal>();
    hood_builder.add_angle(hood_goal_);
    hood_builder.add_profile_params(hood_profile_parameters_offset);
    flatbuffers::Offset<control_loops::superstructure::HoodGoal> hood_offset =
        hood_builder.Finish();

    control_loops::superstructure::ShooterGoal::Builder shooter_builder =
        builder.MakeBuilder<control_loops::superstructure::ShooterGoal>();
    shooter_builder.add_angular_velocity(shooter_velocity_);
    flatbuffers::Offset<control_loops::superstructure::ShooterGoal>
        shooter_offset = shooter_builder.Finish();

    control_loops::superstructure::Goal::Builder goal_builder =
        builder.MakeBuilder<control_loops::superstructure::Goal>();
    goal_builder.add_lights_on(true);
    goal_builder.add_use_vision_for_shots(use_vision_for_shots_);
    goal_builder.add_intake(intake_offset);
    goal_builder.add_indexer(indexer_offset);
    goal_builder.add_turret(turret_offset);
    goal_builder.add_hood(hood_offset);
    goal_builder.add_shooter(shooter_offset);

    flatbuffers::Offset<control_loops::superstructure::Goal> goal_offset =
        goal_builder.Finish();

    control_loops::superstructure::Goal *goal =
        GetMutableTemporaryPointer(*builder.fbb(), goal_offset);

    if (indexer_angular_velocity_ < -0.1) {
      goal->mutable_indexer()->mutate_voltage_rollers(12.0);
      goal->mutable_intake()->mutate_voltage_rollers(6.0);
    } else {
      goal->mutable_indexer()->mutate_voltage_rollers(0.0);
    }

    if (!builder.Send(goal_offset)) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }
};

}  // namespace actors
}  // namespace y2017

#endif  // Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
