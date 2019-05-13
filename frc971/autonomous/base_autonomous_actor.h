#ifndef FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_
#define FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "aos/events/shm-event-loop.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2019/control_loops/drivetrain/target_selector.q.h"

namespace frc971 {
namespace autonomous {

class BaseAutonomousActor
    : public ::aos::common::actions::ActorBase<AutonomousActionQueueGroup> {
 public:
  explicit BaseAutonomousActor(
      ::aos::EventLoop *event_loop, AutonomousActionQueueGroup *s,
      const control_loops::drivetrain::DrivetrainConfig<double> &dt_config);

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
    bool WaitForSplineDistanceRemaining(double distance);

   private:
    friend BaseAutonomousActor;
    SplineHandle(int32_t spline_handle,
                 BaseAutonomousActor *base_autonomous_actor)
        : spline_handle_(spline_handle),
          base_autonomous_actor_(base_autonomous_actor) {}

    int32_t spline_handle_;
    BaseAutonomousActor *base_autonomous_actor_;
  };

  // Represents the direction that we will drive along a spline.
  enum class SplineDirection {
    kForward,
    kBackward,
  };

  // Starts planning the spline, and returns a handle to be used to manipulate
  // it.
  SplineHandle PlanSpline(const ::frc971::MultiSpline &spline,
                          SplineDirection direction);

  void ResetDrivetrain();
  void InitializeEncoders();
  void StartDrive(double distance, double angle, ProfileParameters linear,
                  ProfileParameters angular);

  void WaitUntilDoneOrCanceled(
      ::std::unique_ptr<aos::common::actions::Action> action);
  // Waits for the drive motion to finish.  Returns true if it succeeded, and
  // false if it cancels.
  bool WaitForDriveDone();

  // Returns true if the drive has finished.
  bool IsDriveDone();

  void LineFollowAtVelocity(double velocity, int hint = 0);

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

 private:
  friend class SplineHandle;
  ::aos::Sender<::y2019::control_loops::drivetrain::TargetSelectorHint>
      target_selector_hint_sender_;

  double max_drivetrain_voltage_ = 12.0;

  // Unique counter so we get unique spline handles.
  int spline_handle_ = 0;
  // Last goal spline handle;
  int32_t goal_spline_handle_ = 0;
};

using AutonomousAction =
    ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const AutonomousActionParams &params);

}  // namespace autonomous
}  // namespace frc971

#endif  // FRC971_AUTONOMOUS_BASE_AUTONOMOUS_ACTOR_H_
