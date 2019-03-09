#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
#include "Eigen/Dense"

#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/localizer.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace testing {
class LineFollowDrivetrainTest;
}  // namespace testing

// A drivetrain that permits a velocity input from the driver while controlling
// lateral motion.
// TODO(james): Do we want to allow some driver input on lateral control? It may
// just be too confusing to make work effectively, but it also wouldn't be too
// hard to just allow the driver to specify an offset to the angle/angular
// velocity.
class LineFollowDrivetrain {
 public:
  typedef TypedPose<double> Pose;

  LineFollowDrivetrain(
      const DrivetrainConfig<double> &dt_config,
      TargetSelectorInterface *target_selector);
  // Sets the current goal; the drivetrain queue contains throttle_velocity
  // which is used to command overall robot velocity. The goal_pose is a Pose
  // representing where we are tring to go. This would typically be the Pose of
  // a Target; the positive X-axis in the Pose's frame represents the direction
  // we want to go (we approach the pose from the left-half plane).
  void SetGoal(const ::frc971::control_loops::DrivetrainQueue::Goal &goal);
  // State: [x, y, theta, left_vel, right_vel]
  void Update(const ::Eigen::Matrix<double, 5, 1> &state);
  // Returns whether we set the output. If false, that implies that we do not
  // yet have a target to track into and so some other drivetrain should take
  // over.
  bool SetOutput(
      ::frc971::control_loops::DrivetrainQueue::Output *output);
  void PopulateStatus(
      ::frc971::control_loops::DrivetrainQueue::Status *status) const;

 private:
  // Nominal max voltage.
  // TODO(james): Is there a config for this or anything?
  static constexpr double kMaxVoltage = 12.0;
  // Degree of the polynomial to follow in. Should be strictly greater than 1.
  // A value of 1 would imply that we drive straight to the target (but not hit
  // it straight on, unless we happened to start right in front of it). A value
  // of zero would imply driving straight until we hit the plane of the target.
  // Values between 0 and 1 would imply hitting the target from the side.
  static constexpr double kPolyN = 3.0;
  static_assert(kPolyN > 1.0,
                "We want to hit targets straight on (see above comments).");

  double GoalTheta(const ::Eigen::Matrix<double, 5, 1> &state) const;
  double GoalThetaDot(const ::Eigen::Matrix<double, 5, 1> &state) const;

  const DrivetrainConfig<double> dt_config_;
  // TODO(james): These should probably be factored out somewhere.
  const ::Eigen::DiagonalMatrix<double, 3> Q_ =
      (::Eigen::DiagonalMatrix<double, 3>().diagonal()
           << 1.0 / ::std::pow(0.01, 2),
       1.0 / ::std::pow(0.1, 2), 1.0 / ::std::pow(10.0, 2))
          .finished()
          .asDiagonal();
  const ::Eigen::DiagonalMatrix<double, 2> R_ =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();
  // The matrices we use for the linear controller.
  // State for these is [theta, linear_velocity, angular_velocity]
  const ::Eigen::Matrix<double, 3, 3> A_d_;
  const ::Eigen::Matrix<double, 3, 2> B_d_;
  const ::Eigen::Matrix<double, 2, 3> K_;
  const ::Eigen::Matrix<double, 2, 3> Kff_;

  TargetSelectorInterface *target_selector_;
  bool freeze_target_ = false;
  bool have_target_ = false;
  Pose target_pose_;
  double goal_velocity_ = 0.0;

  // Voltage output to apply
  ::Eigen::Matrix<double, 2, 1> U_;

  friend class testing::LineFollowDrivetrainTest;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
