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
constexpr double kAngleEpsilon = 0.07;

}  // namespace

ClawActor::ClawActor(actors::ClawActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::ClawActionQueueGroup>(s),
      profile_(::aos::controls::kLoopFrequency) {}

bool ClawActor::Iterate(const ClawParams &params) {
  const double goal_angle = params.angle;

  ::Eigen::Matrix<double, 2, 1> goal_state =
      profile_.Update(goal_angle, 0.0);

  auto message = control_loops::claw_queue.goal.MakeMessage();
  message->angle = goal_state(0, 0);
  message->angular_velocity = goal_state(1, 0);
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

  if (::std::abs(goal_angle - current_angle) < kAngleEpsilon &&
      ::std::abs(goal_angle - goal_state(0, 0)) < 0.0000001) {
    return true;
  }

  return false;
}

bool ClawActor::RunAction(const ClawParams &params) {
  control_loops::claw_queue.status.FetchLatest();
  if (control_loops::claw_queue.status.get()) {
    if (!control_loops::claw_queue.status->zeroed) {
      LOG(ERROR, "We are not running actions on an unzeroed claw!\n");
      return false;
    }
    Eigen::Matrix<double, 2, 1> current;
    current.setZero();
    current << control_loops::claw_queue.status->goal_angle, 0.0;

    // Re-initialize the profile to start where we currently are.
    profile_.MoveCurrentState(current);

    // Update the parameters.
    profile_.set_maximum_velocity(params.max_velocity);
    profile_.set_maximum_acceleration(params.max_acceleration);

  } else {
    LOG(ERROR, "No claw status!\n");
    return false;
  }

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
