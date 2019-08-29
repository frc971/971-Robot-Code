#ifndef Y2019_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2019_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"

namespace y2019 {
namespace actors {

using ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;

namespace superstructure = y2019::control_loops::superstructure;

struct ElevatorWristPosition {
  double elevator;
  double wrist;
};

class AutonomousActor : public ::frc971::autonomous::BaseAutonomousActor {
 public:
  explicit AutonomousActor(::aos::EventLoop *event_loop);

  bool RunAction(
      const ::frc971::autonomous::AutonomousActionParams *params) override;

 private:
  void Reset(bool is_left);

  double elevator_goal_ = 0.0;
  double wrist_goal_ = 0.0;
  double intake_goal_ = 0.0;

  bool suction_on_ = true;
  int suction_gamepiece_ = 1;

  double elevator_max_velocity_ = 0.0;
  double elevator_max_acceleration_ = 0.0;
  double wrist_max_velocity_ = 0.0;
  double wrist_max_acceleration_ = 0.0;

  void set_elevator_goal(double elevator_goal) {
    elevator_goal_ = elevator_goal;
  }
  void set_wrist_goal(double wrist_goal) { wrist_goal_ = wrist_goal; }
  void set_intake_goal(double intake_goal) { intake_goal_ = intake_goal; }

  void set_suction_goal(bool on, int gamepiece_mode) {
    suction_on_ = on;
    suction_gamepiece_ = gamepiece_mode;
  }

  void set_elevator_max_velocity(double elevator_max_velocity) {
    elevator_max_velocity_ = elevator_max_velocity;
  }
  void set_elevator_max_acceleration(double elevator_max_acceleration) {
    elevator_max_acceleration_ = elevator_max_acceleration;
  }
  void set_wrist_max_velocity(double wrist_max_velocity) {
    wrist_max_velocity_ = wrist_max_velocity;
  }
  void set_wrist_max_acceleration(double wrist_max_acceleration) {
    wrist_max_acceleration_ = wrist_max_acceleration;
  }

  void set_elevator_wrist_goal(ElevatorWristPosition goal) {
    set_elevator_goal(goal.elevator);
    set_wrist_goal(goal.wrist);
  }

  void SendSuperstructureGoal() {
    auto builder = superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        elevator_offset;

    {
      frc971::ProfileParameters::Builder profile_params_builder =
          builder.MakeBuilder<frc971::ProfileParameters>();
      profile_params_builder.add_max_velocity(elevator_max_velocity_);
      profile_params_builder.add_max_acceleration(elevator_max_acceleration_);

      flatbuffers::Offset<frc971::ProfileParameters> profile_params_offset =
          profile_params_builder.Finish();

      StaticZeroingSingleDOFProfiledSubsystemGoal::Builder elevator_builder =
          builder.MakeBuilder<StaticZeroingSingleDOFProfiledSubsystemGoal>();

      elevator_builder.add_unsafe_goal(elevator_goal_);
      elevator_builder.add_profile_params(profile_params_offset);

      elevator_offset = elevator_builder.Finish();
    }

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        wrist_offset;

    {
      frc971::ProfileParameters::Builder profile_params_builder =
          builder.MakeBuilder<frc971::ProfileParameters>();
      profile_params_builder.add_max_velocity(wrist_max_velocity_);
      profile_params_builder.add_max_acceleration(wrist_max_acceleration_);

      flatbuffers::Offset<frc971::ProfileParameters> profile_params_offset =
          profile_params_builder.Finish();

      StaticZeroingSingleDOFProfiledSubsystemGoal::Builder wrist_builder =
          builder.MakeBuilder<StaticZeroingSingleDOFProfiledSubsystemGoal>();

      wrist_builder.add_unsafe_goal(wrist_goal_);
      wrist_builder.add_profile_params(profile_params_offset);

      wrist_offset = wrist_builder.Finish();
    }

    flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
        intake_offset;

    {
      StaticZeroingSingleDOFProfiledSubsystemGoal::Builder intake_builder =
          builder.MakeBuilder<StaticZeroingSingleDOFProfiledSubsystemGoal>();

      intake_builder.add_unsafe_goal(intake_goal_);

      intake_offset = intake_builder.Finish();
    }

    flatbuffers::Offset<superstructure::SuctionGoal> suction_offset;

    {
      superstructure::SuctionGoal::Builder suction_builder =
          builder.MakeBuilder<superstructure::SuctionGoal>();

      suction_builder.add_grab_piece(suction_on_);
      suction_builder.add_gamepiece_mode(suction_gamepiece_);

      suction_offset = suction_builder.Finish();
    }

    superstructure::Goal::Builder superstructure_builder =
        builder.MakeBuilder<superstructure::Goal>();

    superstructure_builder.add_elevator(elevator_offset);
    superstructure_builder.add_wrist(wrist_offset);
    superstructure_builder.add_intake(intake_offset);
    superstructure_builder.add_suction(suction_offset);

    if (!builder.Send(superstructure_builder.Finish())) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

  bool IsSucked() {
    superstructure_status_fetcher_.Fetch();

    if (superstructure_status_fetcher_.get()) {
      return superstructure_status_fetcher_->has_piece();
    }
    return false;
  }

  bool WaitForGamePiece() {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);

    while (true) {
      if (ShouldCancel()) {
        return false;
      }
      phased_loop.SleepUntilNext();
      if (IsSucked()) {
        return true;
      }
    }
  }

  bool WaitForMilliseconds(std::chrono::milliseconds wait) {
    ::aos::monotonic_clock::time_point end_time = monotonic_now() + wait;

    while (monotonic_now() < end_time) {
      if (ShouldCancel()) {
        return false;
      }
      // TODO(james): Allow non-multiples of 5.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(5));
    }
    return true;
  }

  bool IsSuperstructureDone() {
    superstructure_status_fetcher_.Fetch();

    double kElevatorTolerance = 0.01;
    double kWristTolerance = 0.05;

    if (superstructure_status_fetcher_.get()) {
      const bool elevator_at_goal =
          ::std::abs(elevator_goal_ -
                     superstructure_status_fetcher_->elevator()->position()) <
          kElevatorTolerance;

      const bool wrist_at_goal =
          ::std::abs(wrist_goal_ -
                     superstructure_status_fetcher_->wrist()->position()) <
          kWristTolerance;

      return elevator_at_goal && wrist_at_goal;
    }
    return false;
  }

  bool WaitForSuperstructureDone() {
    ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                        event_loop()->monotonic_now(),
                                        ::std::chrono::milliseconds(5) / 2);

    while (true) {
      if (ShouldCancel()) {
        return false;
      }
      phased_loop.SleepUntilNext();
      superstructure_status_fetcher_.Fetch();
      if (IsSuperstructureDone()) {
        return true;
      }
    }
  }

  // Waits until the robot's x > x.
  bool WaitForDriveXGreater(double x);

  // Waits until y is within y of zero.
  bool WaitForDriveYCloseToZero(double y);

  ::aos::Sender<::frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;
  ::aos::Sender<::y2019::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  ::aos::Fetcher<::y2019::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
};

}  // namespace actors
}  // namespace y2019

#endif  // Y2019_ACTORS_AUTONOMOUS_ACTOR_H_
