#include "y2016/actors/vision_align_actor.h"

#include <chrono>
#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/commonmath.h"
#include "aos/common/time.h"

#include "y2016/actors/vision_align_actor.h"
#include "y2016/constants.h"
#include "y2016/vision/vision.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;

VisionAlignActor::VisionAlignActor(actors::VisionAlignActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::VisionAlignActionQueueGroup>(s) {}

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
    if (!::y2016::vision::vision_status.FetchLatest()) {
      continue;
    }
    const auto &vision_status = *::y2016::vision::vision_status;

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
             .steering(0.0)
             .throttle(0.0)
             .highgear(false)
             .quickturn(false)
             .control_loop_driving(true)
             .left_goal(left_current + side_distance_change)
             .right_goal(right_current - side_distance_change)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
             .Send()) {
      LOG(WARNING, "sending drivetrain goal failed\n");
    }
  }

  LOG(INFO, "Done moving\n");
  return true;
}

::std::unique_ptr<VisionAlignAction> MakeVisionAlignAction(
    const ::y2016::actors::VisionAlignActionParams &params) {
  return ::std::unique_ptr<VisionAlignAction>(
      new VisionAlignAction(&::y2016::actors::vision_align_action, params));
}

}  // namespace actors
}  // namespace y2016
