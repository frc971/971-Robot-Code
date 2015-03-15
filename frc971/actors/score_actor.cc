#include "frc971/actors/score_actor.h"

#include <cmath>

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/phased_loop.h"
#include "frc971/constants.h"
#include "frc971/control_loops/fridge/fridge.q.h"

using ::frc971::control_loops::fridge_queue;

namespace frc971 {
namespace actors {

namespace {
const double kUpperMoveHeight = 0.2;
const double kBeginHorizontalMoveHeight = 0.04;
const double kHorizontalMoveTarget = -0.6;
const double kClearedStopperDistance = 0.2;
const double kAboveFloorHeight = 0.1;
const double kPlaceHeight = -0.05;
const double kHomeReturnHeight = 0.1;
}  // namespace

ScoreActor::ScoreActor(ScoreActionQueueGroup* queues)
    : aos::common::actions::ActorBase<ScoreActionQueueGroup>(queues),
      kinematics_(constants::GetValues().fridge.arm_length,
                  constants::GetValues().fridge.elevator.upper_limit,
                  constants::GetValues().fridge.elevator.lower_limit,
                  constants::GetValues().fridge.arm.upper_limit,
                  constants::GetValues().fridge.arm.lower_limit) {}

bool ScoreActor::RunAction(const ScoreParams& params) {
  // TODO(pschrader): Break these into completely separate actions.
  if (params.place_the_stack) {
    return PlaceTheStack(params);
  } else {
    return MoveStackIntoPosition(params);
  }
}

bool ScoreActor::MoveStackIntoPosition(const ScoreParams& /*params*/) {
  // TODO(pschrader): Assuming right now we start at 0, 0.

  // Move the fridge up a little bit.
  if (!SendGoal(0.0, kUpperMoveHeight, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    // Once we're above a certain height we can start moving backwards.
    if (CurrentHeight() > kBeginHorizontalMoveHeight) {
      break;
    }
  }

  // Move the fridge over.
  if (!SendGoal(kHorizontalMoveTarget, kUpperMoveHeight, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    // Once we've cleared the tote stopper, then we can start moving down.
    if (CurrentX() > kClearedStopperDistance) {
      break;
    }
  }

  // Once we're a bit out, start moving the fridge down.
  if (!SendGoal(kHorizontalMoveTarget, kAboveFloorHeight, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    // Wait until we get to just the right height.
    if (NearGoal(kHorizontalMoveTarget, kAboveFloorHeight)) {
      break;
    }
  }

  if (ShouldCancel()) return true;

  return true;
}

bool ScoreActor::PlaceTheStack(const ScoreParams& /*params*/) {
  // Once the fridge is way out, put it on the ground.
  if (!SendGoal(kHorizontalMoveTarget, kPlaceHeight, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    // Once we're on the ground, move on to releasing the grabber.
    if (NearGoal(kHorizontalMoveTarget, kPlaceHeight)) {
      break;
    }
  }

  if (ShouldCancel()) return true;

  // Release the grabbers.
  if (!SendGoal(kHorizontalMoveTarget, kPlaceHeight, false)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  if (ShouldCancel()) return true;

  // Go back to the home position.
  if (!SendGoal(0.0, kHomeReturnHeight, false)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    if (NearGoal(0.0, kHomeReturnHeight)) {
      break;
    }
  }

  return true;
}

double ScoreActor::CurrentHeight() {
  fridge_queue.status.FetchLatest();

  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return false;
  }

  ::aos::util::ElevatorArmKinematics::KinematicResult results;
  kinematics_.ForwardKinematic(fridge_queue.status->height,
                               fridge_queue.status->angle, 0.0, 0.0, &results);

  return results.fridge_h;
}

double ScoreActor::CurrentX() {
  fridge_queue.status.FetchLatest();

  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return false;
  }

  ::aos::util::ElevatorArmKinematics::KinematicResult results;
  kinematics_.ForwardKinematic(fridge_queue.status->height,
                               fridge_queue.status->angle, 0.0, 0.0, &results);

  return results.fridge_x;
}

bool ScoreActor::SendGoal(double x, double y, bool grabbers_enabled) {
  auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();

  // new_fridge_goal->max_velocity = elevator_parameters.velocity;
  // new_fridge_goal->max_acceleration = elevator_parameters.acceleration;
  new_fridge_goal->x = x;
  new_fridge_goal->y = y;
  new_fridge_goal->profiling_type = 1;
  // new_fridge_goal->velocity = velocity;
  // new_fridge_goal->max_angular_velocity = arm_parameters.velocity;
  // new_fridge_goal->max_angular_acceleration = arm_parameters.acceleration;
  // new_fridge_goal->angle = angle;
  new_fridge_goal->angular_velocity = 0.0;
  new_fridge_goal->grabbers.top_front = grabbers_enabled;
  new_fridge_goal->grabbers.top_back = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_front = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_back = grabbers_enabled;

  return new_fridge_goal.Send();
}

bool ScoreActor::NearGoal(double x, double y) {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return false;
  }

  ::aos::util::ElevatorArmKinematics::KinematicResult results;
  ::aos::util::ElevatorArmKinematics::KinematicResult goal_results;
  kinematics_.ForwardKinematic(fridge_queue.status->height,
                               fridge_queue.status->angle, 0.0, 0.0, &results);
  kinematics_.ForwardKinematic(fridge_queue.status->goal_height,
                               fridge_queue.status->goal_angle, 0.0, 0.0,
                               &goal_results);

  return (::std::abs(results.fridge_x - x) < 0.020 &&
          ::std::abs(results.fridge_h - y) < 0.020 &&
          ::std::abs(goal_results.fridge_x - x) < 0.0001 &&
          ::std::abs(goal_results.fridge_h - y) < 0.0001);
}

::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams& params) {
  return ::std::unique_ptr<ScoreAction>(
      new ScoreAction(&::frc971::actors::score_action, params));
}

}  // namespace actors
}  // namespace frc971
