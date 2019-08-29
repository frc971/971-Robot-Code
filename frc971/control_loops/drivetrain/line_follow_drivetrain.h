#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
#include "Eigen/Dense"

#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/profiled_subsystem_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace testing {
class LineFollowDrivetrainTest;
}  // namespace testing

// A drivetrain that permits a velocity input from the driver while controlling
// lateral motion.
class LineFollowDrivetrain {
 public:
  typedef TypedPose<double> Pose;

  LineFollowDrivetrain(const DrivetrainConfig<double> &dt_config,
                       TargetSelectorInterface *target_selector);
  // Sets the current goal; the drivetrain queue contains throttle_velocity
  // which is used to command overall robot velocity. The goal_pose is a Pose
  // representing where we are trying to go. This would typically be the Pose of
  // a Target; the positive X-axis in the Pose's frame represents the direction
  // we want to go (we approach the pose from the left-half plane).
  void SetGoal(::aos::monotonic_clock::time_point now,
               const ::frc971::control_loops::drivetrain::Goal *goal);
  // State: [x, y, theta, left_vel, right_vel]
  void Update(::aos::monotonic_clock::time_point now,
              const ::Eigen::Matrix<double, 5, 1> &state);
  // Returns whether we set the output. If false, that implies that we do not
  // yet have a target to track into and so some other drivetrain should take
  // over.
  bool SetOutput(
      ::frc971::control_loops::drivetrain::OutputT *output);

  flatbuffers::Offset<LineFollowLogging> PopulateStatus(
      aos::Sender<drivetrain::Status>::Builder *line_follow_logging_builder)
      const;

 private:
  // Nominal max voltage.
  // TODO(james): Is there a config for this or anything?
  static constexpr double kMaxVoltage = 12.0;

  double GoalTheta(const ::Eigen::Matrix<double, 5, 1> &abs_state,
                   double relative_y_offset, double velocity_sign);

  const DrivetrainConfig<double> dt_config_;
  // TODO(james): These should probably be factored out somewhere.
  // TODO(james): This controller is not actually asymptotically stable, due to
  // the varying goal theta.
  const ::Eigen::DiagonalMatrix<double, 3> Q_ =
      (::Eigen::DiagonalMatrix<double, 3>().diagonal()
           << 1.0 / ::std::pow(0.1, 2),
       1.0 / ::std::pow(1.0, 2), 1.0 / ::std::pow(1.0, 2))
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
  // The amount to shift the center of the goal target side-to-side, to give the
  // driver an input that lets them account for panels that are offset on the
  // grabber.
  double side_adjust_ = 0.0;  // meters
  double velocity_sign_ = 1.0;

  // The last time at which we saw an enable button.
  ::aos::monotonic_clock::time_point last_enable_;
  // The time at which we first acquired the current target.
  ::aos::monotonic_clock::time_point start_of_target_acquire_;
  // Most recent relative pose to target, used for debugging.
  Pose relative_pose_;
  // Current goal state, used for debugging.
  ::Eigen::Matrix<double, 3, 1> controls_goal_;

  // Voltage output to apply
  ::Eigen::Matrix<double, 2, 1> U_;

  friend class testing::LineFollowDrivetrainTest;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_LINE_FOLLOW_DRIVETRAIN_H_
