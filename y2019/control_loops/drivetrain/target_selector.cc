#include "y2019/control_loops/drivetrain/target_selector.h"

#include "aos/json_to_flatbuffer.h"

namespace y2019 {
namespace control_loops {

constexpr double TargetSelector::kFakeFov;

TargetSelector::TargetSelector(::aos::EventLoop *event_loop)
    : front_viewer_({&robot_pose_, {0.0, 0.0, 0.0}, 0.0}, kFakeFov, fake_noise_,
                    constants::Field().targets(), {}),
      back_viewer_({&robot_pose_, {0.0, 0.0, 0.0}, M_PI}, kFakeFov, fake_noise_,
                   constants::Field().targets(), {}),
      hint_fetcher_(event_loop->MakeFetcher<drivetrain::TargetSelectorHint>(
          "/drivetrain")),
      superstructure_goal_fetcher_(
          event_loop->MakeFetcher<superstructure::Goal>("/superstructure")) {}

bool TargetSelector::UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                                     double command_speed) {
  if (::std::abs(command_speed) < kMinDecisionSpeed) {
    return false;
  }
  if (superstructure_goal_fetcher_.Fetch()) {
    ball_mode_ = superstructure_goal_fetcher_->suction()->gamepiece_mode() == 0;
  }
  if (hint_fetcher_.Fetch()) {
    VLOG(1) << "selector_hint: " << aos::FlatbufferToJson(hint_fetcher_.get());
    // suggested_target is unsigned so we don't check for >= 0.
    if (hint_fetcher_->suggested_target() < drivetrain::SelectionHint::MAX) {
      target_hint_ = hint_fetcher_->suggested_target();
    } else {
      AOS_LOG(ERROR, "Got invalid suggested target.\n");
    }
  }
  *robot_pose_.mutable_pos() << state.x(), state.y(), 0.0;
  robot_pose_.set_theta(state(2, 0));
  ::aos::SizedArray<FakeCamera::TargetView,
                    y2019::constants::Field::kNumTargets>
      target_views;
  if (command_speed > 0) {
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
    // Skip targets that aren't viable for going to (e.g., on the opposite side
    // of the field).
    // TODO(james): Support ball vs. hatch mode filtering.
    if (view.target->goal_type() == Target::GoalType::kNone ||
        view.target->goal_type() == (ball_mode_ ? Target::GoalType::kHatches
                                                : Target::GoalType::kBalls)) {
      continue;
    }
    switch (target_hint_) {
      case drivetrain::SelectionHint::NEAR_SHIP:
        if (view.target->target_type() !=
            Target::TargetType::kNearSideCargoBay) {
          continue;
        }
        break;
      case drivetrain::SelectionHint::MID_SHIP:
        if (view.target->target_type() !=
            Target::TargetType::kMidSideCargoBay) {
          continue;
        }
        break;
      case drivetrain::SelectionHint::FAR_SHIP:
        if (view.target->target_type() !=
            Target::TargetType::kFarSideCargoBay) {
          continue;
        }
        break;
      case drivetrain::SelectionHint::FAR_ROCKET:
        if (view.target->target_type() != Target::TargetType::kFarRocket) {
          continue;
        }
        break;
      case drivetrain::SelectionHint::NONE:
      default:
        break;
    }
    if (view.noise.distance < largest_target_noise) {
      target_pose_ = view.target->pose();
      // If we are in ball mode, use a radius of zero.
      target_radius_ = ball_mode_ ? 0.0 : view.target->radius();
      largest_target_noise = view.noise.distance;
    }
  }
  return true;
}

}  // namespace control_loops
}  // namespace y2019
