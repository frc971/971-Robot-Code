#ifndef FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_
#define FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/shm_event_loop.h"
#include "frc971/autonomous/auto_generated.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2019/control_loops/drivetrain/target_selector_generated.h"

namespace frc971 {
namespace autonomous {

class BaseAutonomousActor : public ::aos::common::actions::ActorBase<Goal> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<Goal> Factory;

  explicit BaseAutonomousActor(
      ::aos::EventLoop *event_loop,
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/autonomous");
  }

 protected:
  class SplineHandle {
   public:
    bool IsPlanned();
    bool WaitForPlan();
    void Start();

    bool IsDone();
    bool WaitForDone();

    // Whether there is less than a certain distance, in meters, remaining in
    // the current spline.
    bool SplineDistanceRemaining(double distance);
    bool SplineDistanceTraveled(double distance);
    bool WaitForSplineDistanceRemaining(double distance);
    bool WaitForSplineDistanceTraveled(double distance);

    // Returns [x, y, theta] position of the start.
    const Eigen::Vector3d &starting_position() const { return spline_start_; }

   private:
    friend BaseAutonomousActor;
    SplineHandle(int32_t spline_handle,
                 BaseAutonomousActor *base_autonomous_actor,
                 const Eigen::Vector3d &start)
        : spline_handle_(spline_handle),
          base_autonomous_actor_(base_autonomous_actor),
          spline_start_(start) {}

    int32_t spline_handle_;
    BaseAutonomousActor *base_autonomous_actor_;

    // Starting [x, y, theta] position of the spline.
    Eigen::Vector3d spline_start_;
  };

  // Represents the direction that we will drive along a spline.
  enum class SplineDirection {
    kForward,
    kBackward,
  };

  // Starts planning the spline, and returns a handle to be used to manipulate
  // it.
  SplineHandle PlanSpline(
      std::function<flatbuffers::Offset<frc971::MultiSpline>(
          aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
              *builder)> &&multispline_builder,
      SplineDirection direction);

  void ResetDrivetrain();
  void InitializeEncoders();
  void StartDrive(double distance, double angle, ProfileParametersT linear,
                  ProfileParametersT angular);
  void ApplyThrottle(double throttle);

  void WaitUntilDoneOrCanceled(
      ::std::unique_ptr<aos::common::actions::Action> action);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();

  // Returns true if the drive has finished.
  bool IsDriveDone();

  // Returns the current x, y, theta of the robot on the field.
  double X();
  double Y();
  double Theta();

  void LineFollowAtVelocity(
      double velocity,
      y2019::control_loops::drivetrain::SelectionHint hint =
          y2019::control_loops::drivetrain::SelectionHint::NONE);

  // Waits until the robot is pitched up above the specified angle, or the move
  // finishes.  Returns true on success, and false if it cancels.
  bool WaitForAboveAngle(double angle);
  bool WaitForBelowAngle(double angle);
  bool WaitForMaxBy(double angle);

  // Waits until the profile and distance is within distance and angle of the
  // goal.  Returns true on success, and false when canceled.
  bool WaitForDriveNear(double distance, double angle);

  bool WaitForDriveProfileNear(double tolerance);
  bool WaitForDriveProfileDone();

  bool WaitForTurnProfileNear(double tolerance);
  bool WaitForTurnProfileDone();

  void set_max_drivetrain_voltage(double max_drivetrain_voltage) {
    max_drivetrain_voltage_ = max_drivetrain_voltage;
  }

  // Returns the distance left to go.
  double DriveDistanceLeft();

  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;

  // Initial drivetrain positions.
  struct InitialDrivetrain {
    double left;
    double right;
  };
  InitialDrivetrain initial_drivetrain_;

  ::aos::Sender<::y2019::control_loops::drivetrain::TargetSelectorHint>
      target_selector_hint_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_sender_;
  ::aos::Sender<::frc971::control_loops::drivetrain::SplineGoal>
      spline_goal_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_fetcher_;

 private:
  friend class SplineHandle;
  double max_drivetrain_voltage_ = 12.0;

  // Unique counter so we get unique spline handles.
  int spline_handle_ = 0;
  // Last goal spline handle;
  int32_t goal_spline_handle_ = 0;
};

}  // namespace autonomous
}  // namespace frc971

#endif  // FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_
