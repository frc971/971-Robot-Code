#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/commonmath.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/actions/actor.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/constants.h"
#include "frc971/actors/claw_actor.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {

// Defines finished.
constexpr double kAngleEpsilon = 0.01;

}  // namespace

ClawActor::ClawActor(actors::ClawActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::ClawActionQueueGroup>(s) {}

bool ClawActor::Iterate(const ClawParams &params) {
  const double goal_angle = params.claw_angle;
  const double goal_velocity = params.claw_max_velocity;

  if (::std::abs(claw_start_angle_ + delta_angle_ - goal_angle) >
      kAngleEpsilon) {
    delta_angle_ += goal_velocity * ::aos::controls::kLoopFrequency.ToSeconds();
  } else {
    delta_angle_ = goal_angle - claw_start_angle_;
  }

  auto message = control_loops::claw_queue.goal.MakeMessage();
  message->angle = claw_start_angle_ + delta_angle_;
  message->angular_velocity = goal_velocity;
  message->intake = params.intake_voltage;
  message->rollers_closed = params.rollers_closed;

  LOG_STRUCT(DEBUG, "Sending claw goal", *message);
  message.Send();

  control_loops::claw_queue.status.FetchLatest();
  if (!control_loops::claw_queue.status.get()) {
    return false;
  }
  const double current_angle = control_loops::claw_queue.status->angle;
  LOG_STRUCT(DEBUG, "Got claw status", *control_loops::claw_queue.status);

  if (::std::abs(goal_angle - current_angle) < kAngleEpsilon) {
    return true;
  }

  return false;
}

bool ClawActor::RunAction(const ClawParams &params) {
  LOG(INFO, "Claw goal (angle, velocity): %f, %f\n", params.claw_angle,
      params.claw_max_velocity);

  control_loops::claw_queue.status.FetchLatest();
  if (control_loops::claw_queue.status.get()) {
    if (!control_loops::claw_queue.status->zeroed) {
      LOG(ERROR, "We are not running actions on an unzeroed claw!\n");
      return false;
    }
    claw_start_angle_ = control_loops::claw_queue.status->angle;
  } else {
    LOG(ERROR, "No claw status!\n");
    return false;
  }

  delta_angle_ = 0.0;
  while (!Iterate(params)) {
    // wait until next Xms tick
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);

    // check if we should stop before we send
    if (ShouldCancel()) return true;
  }

  LOG(INFO, "Claw done moving.\n");
  return true;
}

::std::unique_ptr<ClawAction> MakeClawAction(const ClawParams &p) {
  return ::std::unique_ptr<ClawAction>(
      new ClawAction(&::frc971::actors::claw_action, p));
}

}  // namespace actors
}  // namespace frc971
