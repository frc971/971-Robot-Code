#include "y2019/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"

#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2019/actors/auto_splines.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"

namespace y2019 {
namespace actors {

using ::frc971::ProfileParametersT;
using ::aos::monotonic_clock;
using frc971::control_loops::drivetrain::LocalizerControl;
namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      localizer_control_sender_(
          event_loop->MakeSender<
              ::frc971::control_loops::drivetrain::LocalizerControl>(
              "/drivetrain")),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2019::control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2019::control_loops::superstructure::Status>(
                  "/superstructure")) {}

bool AutonomousActor::WaitForDriveXGreater(double x) {
  AOS_LOG(INFO, "Waiting until x > %f\n", x);
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (drivetrain_status_fetcher_->x() > x) {
      AOS_LOG(INFO, "X at %f\n", drivetrain_status_fetcher_->x());
      return true;
    }
  }
}

bool AutonomousActor::WaitForDriveYCloseToZero(double y) {
  AOS_LOG(INFO, "Waiting until |y| < %f\n", y);
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (::std::abs(drivetrain_status_fetcher_->y()) < y) {
      AOS_LOG(INFO, "Y at %f\n", drivetrain_status_fetcher_->y());
      return true;
    }
  }
}

void AutonomousActor::Reset(bool is_left) {
  const double turn_scalar = is_left ? 1.0 : -1.0;
  elevator_goal_ = 0.01;
  wrist_goal_ = -M_PI / 2.0;
  intake_goal_ = -1.2;

  suction_on_ = false;
  suction_gamepiece_ = 1;

  elevator_max_velocity_ = 0.0;
  elevator_max_acceleration_ = 0.0;
  wrist_max_velocity_ = 0.0;
  wrist_max_acceleration_ = 0.0;

  InitializeEncoders();
  SendSuperstructureGoal();

  {
    auto builder = localizer_control_sender_.MakeBuilder();

    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();
    // Start on the left l2.
    localizer_control_builder.add_x(1.0);
    localizer_control_builder.add_y(1.35 * turn_scalar);
    localizer_control_builder.add_theta(M_PI);
    localizer_control_builder.add_theta_uncertainty(0.00001);
    if (!builder.Send(localizer_control_builder.Finish())) {
      AOS_LOG(ERROR, "Failed to reset localizer.\n");
    }
  }

  // Wait for the drivetrain to run so it has time to reset the heading.
  // Otherwise our drivetrain reset will do a 180 right at the start.
  WaitUntil([this]() { return drivetrain_status_fetcher_.Fetch(); });
  AOS_LOG(INFO, "Heading is %f\n",
          drivetrain_status_fetcher_->estimated_heading());
  InitializeEncoders();
  ResetDrivetrain();
  WaitUntil([this]() { return drivetrain_status_fetcher_.Fetch(); });
  AOS_LOG(INFO, "Heading is %f\n",
          drivetrain_status_fetcher_->estimated_heading());

  ResetDrivetrain();
  InitializeEncoders();
}

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kJumpDrive = MakeProfileParameters(2.0, 3.0);
const ProfileParametersT kDrive = MakeProfileParameters(4.0, 3.0);
const ProfileParametersT kTurn = MakeProfileParameters(5.0, 15.0);

const ElevatorWristPosition kPanelHPIntakeForwrdPos{0.01, M_PI / 2.0};
const ElevatorWristPosition kPanelHPIntakeBackwardPos{0.015, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardMiddlePos{0.75, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardMiddlePos{0.78, -M_PI / 2.0};

const ElevatorWristPosition kPanelBackwardUpperPos{1.50, -M_PI / 2.0};

const ElevatorWristPosition kPanelCargoBackwardPos{0.0, -M_PI / 2.0};

template <typename Functor>
std::function<flatbuffers::Offset<frc971::MultiSpline>(
    aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder *builder)>
BindIsLeft(Functor f, bool is_left) {
  return
      [is_left, f](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                       *builder) { return f(builder, is_left); };
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  const monotonic_clock::time_point start_time = monotonic_now();
  const bool is_left = params->mode() == 0;

  {
    AOS_LOG(INFO, "Starting autonomous action with mode %" PRId32 " %s\n",
            params->mode(), is_left ? "left" : "right");
  }

  const double turn_scalar = is_left ? 1.0 : -1.0;

  Reset(is_left);
  enum class Mode { kTesting, kRocket, kCargoship };
  Mode mode = Mode::kCargoship;
  if (mode == Mode::kRocket) {
    SplineHandle spline1 =
        PlanSpline(BindIsLeft(AutonomousSplines::HabToFarRocketTest, is_left),
                   SplineDirection::kBackward);

    // Grab the disk, jump, wait until we have vacuum, then raise the elevator
    set_elevator_goal(0.010);
    set_wrist_goal(-M_PI / 2.0);
    set_intake_goal(-1.2);
    set_suction_goal(true, 1);
    SendSuperstructureGoal();

    // if planned start the spline and plan the next
    if (!spline1.WaitForPlan()) return true;
    AOS_LOG(INFO, "Planned\n");
    spline1.Start();

    // If suction, move the superstructure to score
    if (!WaitForGamePiece()) return true;
    AOS_LOG(INFO, "Has game piece\n");
    if (!spline1.WaitForSplineDistanceRemaining(3.5)) return true;
    set_elevator_wrist_goal(kPanelBackwardMiddlePos);
    SendSuperstructureGoal();

    if (!spline1.WaitForSplineDistanceRemaining(2.0)) return true;
    set_elevator_wrist_goal(kPanelForwardMiddlePos);
    SendSuperstructureGoal();

    // END SPLINE 1

    if (!spline1.WaitForSplineDistanceRemaining(0.2)) return true;
    LineFollowAtVelocity(1.3,
                         control_loops::drivetrain::SelectionHint::FAR_ROCKET);
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1200))) return true;

    set_suction_goal(false, 1);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(200))) return true;
    LineFollowAtVelocity(-1.0,
                         control_loops::drivetrain::SelectionHint::FAR_ROCKET);
    SplineHandle spline2 =
        PlanSpline(BindIsLeft(AutonomousSplines::FarRocketToHP, is_left),
                   SplineDirection::kBackward);

    if (!WaitForMilliseconds(::std::chrono::milliseconds(150))) return true;
    if (!spline2.WaitForPlan()) return true;
    AOS_LOG(INFO, "Planned\n");
    // Drive back to hp and set the superstructure accordingly
    spline2.Start();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(500))) return true;
    set_elevator_wrist_goal(kPanelHPIntakeBackwardPos);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1000))) return true;
    set_suction_goal(true, 1);
    SendSuperstructureGoal();

    if (!spline2.WaitForSplineDistanceRemaining(1.6)) return true;
    LineFollowAtVelocity(-1.6);

    // As soon as we pick up Panel 2 go score on the back rocket
    if (!WaitForGamePiece()) return true;
    LineFollowAtVelocity(1.5);
    SplineHandle spline3 =
        PlanSpline(BindIsLeft(AutonomousSplines::HPToFarRocket, is_left),
                   SplineDirection::kForward);
    if (!WaitForDriveXGreater(0.50)) return true;
    if (!spline3.WaitForPlan()) return true;
    spline3.Start();
    AOS_LOG(INFO, "Has game piece\n");
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1000))) return true;
    set_elevator_wrist_goal(kPanelBackwardMiddlePos);
    SendSuperstructureGoal();
    if (!WaitForDriveXGreater(7.1)) return true;
    LineFollowAtVelocity(-1.5,
                         control_loops::drivetrain::SelectionHint::FAR_ROCKET);
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1000))) return true;
    set_elevator_wrist_goal(kPanelBackwardUpperPos);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1500))) return true;
    set_suction_goal(false, 1);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(400))) return true;
    LineFollowAtVelocity(1.0,
                         control_loops::drivetrain::SelectionHint::FAR_ROCKET);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(200))) return true;
  } else if (mode == Mode::kCargoship) {
    SplineHandle spline1 = PlanSpline(
        BindIsLeft(AutonomousSplines::HABToSecondCargoShipBay, is_left),
        SplineDirection::kBackward);
    set_elevator_goal(0.01);
    set_wrist_goal(-M_PI / 2.0);
    set_intake_goal(-1.2);
    set_suction_goal(true, 1);
    SendSuperstructureGoal();

    // if planned start the spline and plan the next
    if (!spline1.WaitForPlan()) return true;
    AOS_LOG(INFO, "Planned\n");
    spline1.Start();

    // If suction, move the superstructure to score
    if (!WaitForGamePiece()) return true;
    AOS_LOG(INFO, "Has game piece\n");
    // unstick the hatch panel
    if (!WaitForMilliseconds(::std::chrono::milliseconds(500))) return true;
    set_elevator_goal(0.5);
    set_wrist_goal(-M_PI / 2.0);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(500))) return true;
    set_elevator_wrist_goal(kPanelCargoBackwardPos);
    SendSuperstructureGoal();

    if (!spline1.WaitForSplineDistanceRemaining(0.8)) return true;
    // Line follow in to the first disc.
    LineFollowAtVelocity(-0.9,
                         control_loops::drivetrain::SelectionHint::MID_SHIP);
    if (!WaitForDriveYCloseToZero(1.2)) return true;

    set_suction_goal(false, 1);
    SendSuperstructureGoal();
    AOS_LOG(INFO, "Dropping disc 1 %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));

    if (!WaitForDriveYCloseToZero(1.13)) return true;
    if (!WaitForMilliseconds(::std::chrono::milliseconds(300))) return true;

    LineFollowAtVelocity(0.9,
                         control_loops::drivetrain::SelectionHint::MID_SHIP);
    SplineHandle spline2 = PlanSpline(
        BindIsLeft(AutonomousSplines::SecondCargoShipBayToHP, is_left),
        SplineDirection::kForward);
    if (!WaitForMilliseconds(::std::chrono::milliseconds(400))) return true;
    if (!spline2.WaitForPlan()) return true;
    AOS_LOG(INFO, "Planned\n");
    // Drive back to hp and set the superstructure accordingly
    spline2.Start();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(200))) return true;
    set_elevator_wrist_goal(kPanelHPIntakeForwrdPos);
    SendSuperstructureGoal();
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1000))) return true;
    set_suction_goal(true, 1);
    SendSuperstructureGoal();

    if (!spline2.WaitForSplineDistanceRemaining(1.75)) return true;
    LineFollowAtVelocity(1.5);
    // As soon as we pick up Panel 2 go score on the rocket
    if (!WaitForGamePiece()) return true;
    AOS_LOG(INFO, "Got gamepiece %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    LineFollowAtVelocity(-4.0);
    SplineHandle spline3 = PlanSpline(
        BindIsLeft(AutonomousSplines::HPToThirdCargoShipBay, is_left),
        SplineDirection::kBackward);
    if (!WaitForDriveXGreater(0.55)) return true;
    if (!spline3.WaitForPlan()) return true;
    spline3.Start();
    // Wait until we are a bit out to lift.
    if (!WaitForMilliseconds(::std::chrono::milliseconds(1000))) return true;
    set_elevator_wrist_goal(kPanelCargoBackwardPos);
    SendSuperstructureGoal();

    if (!spline3.WaitForSplineDistanceRemaining(0.7)) return true;
    // Line follow in to the second disc.
    LineFollowAtVelocity(-0.7,
                         control_loops::drivetrain::SelectionHint::FAR_SHIP);
    AOS_LOG(INFO, "Drawing in disc 2 %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    if (!WaitForDriveYCloseToZero(1.2)) return true;

    set_suction_goal(false, 1);
    SendSuperstructureGoal();
    AOS_LOG(INFO, "Dropping disc 2 %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));

    if (!WaitForDriveYCloseToZero(1.13)) return true;
    if (!WaitForMilliseconds(::std::chrono::milliseconds(200))) return true;
    AOS_LOG(INFO, "Backing up %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    LineFollowAtVelocity(0.9,
                         control_loops::drivetrain::SelectionHint::FAR_SHIP);
    if (!WaitForMilliseconds(::std::chrono::milliseconds(400))) return true;
  } else {
    // Grab the disk, wait until we have vacuum, then jump
    set_elevator_goal(0.01);
    set_wrist_goal(-M_PI / 2.0);
    set_intake_goal(-1.2);
    set_suction_goal(true, 1);
    SendSuperstructureGoal();

    if (!WaitForGamePiece()) return true;
    AOS_LOG(INFO, "Has game piece\n");

    StartDrive(-4.0, 0.0, kJumpDrive, kTurn);
    if (!WaitForDriveNear(3.3, 10.0)) return true;
    AOS_LOG(INFO, "Lifting\n");
    set_elevator_goal(0.30);
    SendSuperstructureGoal();

    if (!WaitForDriveNear(2.8, 10.0)) return true;
    AOS_LOG(INFO, "Off the platform\n");

    StartDrive(0.0, 1.00 * turn_scalar, kDrive, kTurn);
    AOS_LOG(INFO, "Turn started\n");
    if (!WaitForSuperstructureDone()) return true;
    AOS_LOG(INFO, "Superstructure done\n");

    if (!WaitForDriveNear(0.7, 10.0)) return true;
    StartDrive(0.0, -0.35 * turn_scalar, kDrive, kTurn);

    AOS_LOG(INFO, "Elevator up\n");
    set_elevator_goal(0.78);
    SendSuperstructureGoal();

    if (!WaitForDriveDone()) return true;
    AOS_LOG(INFO, "Done driving\n");

    if (!WaitForSuperstructureDone()) return true;
  }

  AOS_LOG(INFO, "Done %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  return true;
}

}  // namespace actors
}  // namespace y2019
