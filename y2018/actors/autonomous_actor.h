#ifndef Y2018_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2018_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/event_loop.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2018/control_loops/superstructure/arm/generated_graph.h"
#include "y2018/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2018/control_loops/superstructure/superstructure_status_generated.h"

namespace y2018 {
namespace actors {

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset() {
    roller_voltage_ = 0.0;
    left_intake_angle_ = -3.2;
    right_intake_angle_ = -3.2;
    arm_goal_position_ =
        ::y2018::control_loops::superstructure::arm::NeutralIndex();
    grab_box_ = false;
    open_claw_ = false;
    close_claw_ = false;
    deploy_fork_ = false;
    disable_box_correct_ = false;
    InitializeEncoders();
    ResetDrivetrain();
    SendSuperstructureGoal();
  }

  ::aos::Sender<::y2018::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  ::aos::Fetcher<::y2018::control_loops::superstructure::Status>
      superstructure_status_fetcher_;

  double roller_voltage_ = 0.0;
  double left_intake_angle_ = -3.2;
  double right_intake_angle_ = -3.2;
  uint32_t arm_goal_position_ =
      ::y2018::control_loops::superstructure::arm::NeutralIndex();
  bool grab_box_ = false;
  bool open_claw_ = false;
  bool close_claw_ = false;
  bool deploy_fork_ = false;
  bool disable_box_correct_ = false;

  void set_roller_voltage(double roller_voltage) {
    roller_voltage_ = roller_voltage;
  }
  void set_intake_angle(double intake_angle) {
    set_left_intake_angle(intake_angle);
    set_right_intake_angle(intake_angle);
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
  void set_close_claw(bool close_claw) { close_claw_ = close_claw; }
  void set_deploy_fork(bool deploy_fork) { deploy_fork_ = deploy_fork; }

  void set_disable_box_correct(bool disable_box_correct) {
    disable_box_correct_ = disable_box_correct;
  }

  void SendSuperstructureGoal() {
    auto builder = superstructure_goal_sender_.MakeBuilder();
    control_loops::superstructure::IntakeGoal::Builder intake_goal_builder =
        builder.MakeBuilder<control_loops::superstructure::IntakeGoal>();
    intake_goal_builder.add_roller_voltage(roller_voltage_);
    intake_goal_builder.add_left_intake_angle(left_intake_angle_);
    intake_goal_builder.add_right_intake_angle(right_intake_angle_);

    flatbuffers::Offset<control_loops::superstructure::IntakeGoal>
        intake_offset = intake_goal_builder.Finish();

    control_loops::superstructure::Goal::Builder superstructure_builder =
        builder.MakeBuilder<control_loops::superstructure::Goal>();

    superstructure_builder.add_intake(intake_offset);

    superstructure_builder.add_arm_goal_position(arm_goal_position_);
    superstructure_builder.add_grab_box(grab_box_);
    superstructure_builder.add_open_claw(open_claw_);
    superstructure_builder.add_close_claw(close_claw_);
    superstructure_builder.add_deploy_fork(deploy_fork_);
    superstructure_builder.add_trajectory_override(false);

    if (!builder.Send(superstructure_builder.Finish())) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

  bool ThreeCubeAuto(::aos::monotonic_clock::time_point start_time);
  bool CloseSwitch(::aos::monotonic_clock::time_point start_time,
                   bool left = true);
  bool FarSwitch(::aos::monotonic_clock::time_point start_time,
                 bool drive_behind = true, bool left = true);
  bool FarReadyScale(::aos::monotonic_clock::time_point start_time);
  bool DriveStraight();

  bool FarScale(::aos::monotonic_clock::time_point start_time);

  bool WaitForArmTrajectoryOrDriveClose(double drive_threshold,
                                        double arm_threshold) {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);

    constexpr double kPositionTolerance = 0.02;
    constexpr double kProfileTolerance = 0.001;

    while (true) {
      if (ShouldCancel()) {
        return false;
      }

      superstructure_status_fetcher_.Fetch();
      drivetrain_status_fetcher_.Fetch();
      if (drivetrain_status_fetcher_.get() &&
          superstructure_status_fetcher_.get()) {
        const double left_profile_error =
            (initial_drivetrain_.left -
             drivetrain_status_fetcher_->profiled_left_position_goal());
        const double right_profile_error =
            (initial_drivetrain_.right -
             drivetrain_status_fetcher_->profiled_right_position_goal());

        const double left_error =
            (initial_drivetrain_.left -
             drivetrain_status_fetcher_->estimated_left_position());
        const double right_error =
            (initial_drivetrain_.right -
             drivetrain_status_fetcher_->estimated_right_position());

        const double profile_distance_to_go =
            (left_profile_error + right_profile_error) / 2.0;

        const double distance_to_go = (left_error + right_error) / 2.0;

        // Check superstructure first.
        if (superstructure_status_fetcher_->arm()->current_node() ==
                arm_goal_position_ &&
            superstructure_status_fetcher_->arm()->path_distance_to_go() <
                arm_threshold) {
          AOS_LOG(INFO, "Arm finished first: %f, drivetrain %f distance\n",
                  superstructure_status_fetcher_->arm()->path_distance_to_go(),
                  ::std::abs(distance_to_go));
          return true;
        }

        // Now check drivetrain.
        if (::std::abs(profile_distance_to_go) <
                drive_threshold + kProfileTolerance &&
            ::std::abs(distance_to_go) < drive_threshold + kPositionTolerance) {
          AOS_LOG(INFO,
                  "Drivetrain finished first: arm %f, drivetrain %f distance\n",
                  superstructure_status_fetcher_->arm()->path_distance_to_go(),
                  ::std::abs(distance_to_go));
          return true;
        }
      }
      phased_loop.SleepUntilNext();
    }
  }

  bool WaitForArmTrajectoryClose(double threshold) {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);
    while (true) {
      if (ShouldCancel()) {
        return false;
      }

      superstructure_status_fetcher_.Fetch();
      if (superstructure_status_fetcher_.get()) {
        if (superstructure_status_fetcher_->arm()->current_node() ==
                arm_goal_position_ &&
            superstructure_status_fetcher_->arm()->path_distance_to_go() <
                threshold) {
          return true;
        }
      }
      phased_loop.SleepUntilNext();
    }
  }

  bool WaitForBoxGrabed() {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);
    while (true) {
      if (ShouldCancel()) {
        return false;
      }

      superstructure_status_fetcher_.Fetch();
      if (superstructure_status_fetcher_.get()) {
        if (superstructure_status_fetcher_->arm()->grab_state() == 4) {
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
