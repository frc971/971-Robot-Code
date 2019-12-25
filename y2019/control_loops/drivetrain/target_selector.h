#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_

#include "frc971/control_loops/pose.h"
#include "frc971/control_loops/drivetrain/localizer.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera.h"
#include "y2019/control_loops/drivetrain/target_selector_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"

namespace y2019 {
namespace control_loops {

// A class to identify which target the driver is currently driving towards so
// that we can guide them into the target.
// The current high-level algorithm is to:
// (a) Determine which direction (forwards vs. backwardS) the driver is driving.
// (b) Find the largest target in the X degree field-of-view to the front/back
// of the robot, where X depends on how much of an angle we expect the driver to
// typically want to drive in at.
// (c) Assume that said largest target is the target that the driver wants to
// drive to.
class TargetSelector
    : public ::frc971::control_loops::drivetrain::TargetSelectorInterface {
 public:
  typedef ::frc971::control_loops::TypedPose<double> Pose;
  // For the virtual camera that we use to identify targets, ignore all
  // obstacles and just assume that we have perfect field of view.
  typedef TypedCamera<y2019::constants::Field::kNumTargets,
                      /*num_obstacles=*/0, double> FakeCamera;

  TargetSelector(::aos::EventLoop *event_loop);

  bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                       double command_speed) override;
  Pose TargetPose() const override { return target_pose_; }

  double TargetRadius() const override { return target_radius_; }

 private:
  static constexpr double kFakeFov = M_PI * 0.9;
  // Longitudinal speed at which the robot must be going in order for us to make
  // a decision.
  static constexpr double kMinDecisionSpeed = 0.3;  // m/s
  Pose robot_pose_;
  Pose target_pose_;
  double target_radius_;
  // For the noise of our fake cameras, we only really care about the max
  // distance, which will be the maximum distance we let ourselves guide in
  // from. The distance noise is set so that we can use the camera's estimate of
  // the relative size of the targets.
  FakeCamera::NoiseParameters fake_noise_ = {.max_viewable_distance = 5 /*m*/,
                                             .heading_noise = 0,
                                             .nominal_distance_noise = 1,
                                             .nominal_skew_noise = 0,
                                             .nominal_height_noise = 0};
  FakeCamera front_viewer_;
  FakeCamera back_viewer_;

  ::aos::Fetcher<drivetrain::TargetSelectorHint> hint_fetcher_;
  ::aos::Fetcher<superstructure::Goal> superstructure_goal_fetcher_;

  // Whether we are currently in ball mode.
  bool ball_mode_ = false;
  drivetrain::SelectionHint target_hint_ = drivetrain::SelectionHint::NONE;
};

}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_TARGET_SELECTOR_H_
