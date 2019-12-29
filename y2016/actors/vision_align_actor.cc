#include "y2016/actors/vision_align_actor.h"

#include <chrono>
#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/commonmath.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "aos/util/trapezoid_profile.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/vision/vision_generated.h"

namespace y2016 {
namespace actors {

VisionAlignActor::VisionAlignActor(::aos::EventLoop *event_loop)
    : aos::common::actions::ActorBase<vision_align_action::Goal>(
          event_loop, "/vision_align_action"),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2016::vision::VisionStatus>(
              "/superstructure")),
      drivetrain_goal_sender_(
          event_loop->MakeSender<::frc971::control_loops::drivetrain::Goal>(
              "/drivetrain")) {}

bool VisionAlignActor::RunAction(
    const vision_align_action::VisionAlignActionParams * /*params*/) {
  const double robot_radius =
      control_loops::drivetrain::GetDrivetrainConfig().robot_radius;
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    const int iterations = phased_loop.SleepUntilNext();
    if (iterations != 1) {
      AOS_LOG(WARNING, "vision align actor skipped %d iterations\n",
              iterations - 1);
    }

    if (ShouldCancel()) {
      return true;
    }
    if (!vision_status_fetcher_.Fetch()) {
      continue;
    }
    if (!vision_status_fetcher_->left_image_valid() ||
        !vision_status_fetcher_->right_image_valid()) {
      continue;
    }

    const double side_distance_change =
        vision_status_fetcher_->horizontal_angle() * robot_radius;
    if (!::std::isfinite(side_distance_change)) {
      continue;
    }

    const double left_current =
        vision_status_fetcher_->drivetrain_left_position();
    const double right_current =
        vision_status_fetcher_->drivetrain_right_position();

    auto builder = drivetrain_goal_sender_.MakeBuilder();

    frc971::control_loops::drivetrain::Goal::Builder goal_builder =
        builder.MakeBuilder<frc971::control_loops::drivetrain::Goal>();
    goal_builder.add_wheel(0.0);
    goal_builder.add_throttle(0.0);
    goal_builder.add_highgear(false);
    goal_builder.add_quickturn(false);
    goal_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(left_current + side_distance_change);
    goal_builder.add_right_goal(right_current - side_distance_change);

    if (!builder.Send(goal_builder.Finish())) {
      AOS_LOG(WARNING, "sending drivetrain goal failed\n");
    }
  }

  AOS_LOG(INFO, "Done moving\n");
  return true;
}

}  // namespace actors
}  // namespace y2016
