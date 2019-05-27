#include "y2016/actors/vision_align_actor.h"

#include <chrono>
#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/util/phased_loop.h"
#include "aos/logging/logging.h"
#include "aos/util/trapezoid_profile.h"
#include "aos/commonmath.h"
#include "aos/time/time.h"

#include "y2016/actors/vision_align_actor.h"
#include "y2016/constants.h"
#include "y2016/vision/vision.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;

VisionAlignActor::VisionAlignActor(::aos::EventLoop *event_loop)
    : aos::common::actions::ActorBase<actors::VisionAlignActionQueueGroup>(
          event_loop, ".y2016.actors.vision_align_action"),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2016::vision::VisionStatus>(
              ".y2016.vision.vision_status")) {}

bool VisionAlignActor::RunAction(
    const actors::VisionAlignActionParams & /*params*/) {
  const double robot_radius =
      control_loops::drivetrain::GetDrivetrainConfig().robot_radius;
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    const int iterations = phased_loop.SleepUntilNext();
    if (iterations != 1) {
      LOG(WARNING, "vision align actor skipped %d iterations\n",
          iterations - 1);
    }

    if (ShouldCancel()) {
      return true;
    }
    if (!vision_status_fetcher_.Fetch()) {
      continue;
    }
    const auto &vision_status = *vision_status_fetcher_;

    if (!vision_status.left_image_valid || !vision_status.right_image_valid) {
      continue;
    }

    const double side_distance_change =
        vision_status.horizontal_angle * robot_radius;
    if (!::std::isfinite(side_distance_change)) {
      continue;
    }

    const double left_current = vision_status.drivetrain_left_position;
    const double right_current = vision_status.drivetrain_right_position;

    if (!drivetrain_queue.goal.MakeWithBuilder()
             .wheel(0.0)
             .throttle(0.0)
             .highgear(false)
             .quickturn(false)
             .controller_type(1)
             .left_goal(left_current + side_distance_change)
             .right_goal(right_current - side_distance_change)
             .Send()) {
      LOG(WARNING, "sending drivetrain goal failed\n");
    }
  }

  LOG(INFO, "Done moving\n");
  return true;
}

}  // namespace actors
}  // namespace y2016
