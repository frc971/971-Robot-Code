#include "y2019/control_loops/drivetrain/target_selector.h"

namespace y2019 {
namespace control_loops {

constexpr double TargetSelector::kFakeFov;

TargetSelector::TargetSelector()
    : front_viewer_({&robot_pose_, {0.0, 0.0, 0.0}, 0.0}, kFakeFov, fake_noise_,
                    constants::Field().targets(), {}),
      back_viewer_({&robot_pose_, {0.0, 0.0, 0.0}, M_PI}, kFakeFov, fake_noise_,
                   constants::Field().targets(), {}) {}

bool TargetSelector::UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state) {
  const double speed = (state(3, 0) + state(4, 0)) / 2.0;
  if (::std::abs(speed) < kMinDecisionSpeed) {
    return false;
  }
  *robot_pose_.mutable_pos() << state.x(), state.y(), 0.0;
  robot_pose_.set_theta(state(2, 0));
  ::aos::SizedArray<FakeCamera::TargetView,
                    y2019::constants::Field::kNumTargets>
      target_views;
  if (speed > 0) {
    target_views = front_viewer_.target_views();
  } else {
    target_views = back_viewer_.target_views();
  }

  if (target_views.empty()) {
    // We can't see any targets...
    return false;
  }

  // Choose the target that has the smallest distance noise (currently, this
  // means the largest target in the camera view).
  double largest_target_noise = ::std::numeric_limits<double>::infinity();
  for (const auto &view : target_views) {
    if (view.noise.distance < largest_target_noise) {
      target_pose_ = view.target->pose();
      largest_target_noise = view.noise.distance;
    }
  }
  return true;
}

}  // namespace control_loops
}  // namespace y2019
