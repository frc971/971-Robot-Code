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

const double kMaxXVelocity = 0.45;
const double kMaxYVelocity = 0.20;
const double kMaxXAcceleration = 0.5;
const double kMaxYAcceleration = 0.5;

}  // namespace

ScoreActor::ScoreActor(ScoreActionQueueGroup* queues)
    : aos::common::actions::ActorBase<ScoreActionQueueGroup>(queues),
      kinematics_(constants::GetValues().fridge.arm_length,
                  constants::GetValues().fridge.elevator.upper_limit,
                  constants::GetValues().fridge.elevator.lower_limit,
                  constants::GetValues().fridge.arm.upper_limit,
                  constants::GetValues().fridge.arm.lower_limit) {}

bool ScoreActor::RunAction(const ScoreParams& params) {
  if (params.move_the_stack) {
    LOG(INFO, "moving stack\n");
    if (!MoveStackIntoPosition(params)) return false;
    LOG(INFO, "done moving stack\n");
    if (ShouldCancel()) return true;
  }
  if (params.place_the_stack) {
    LOG(INFO, "placing stack\n");
    if (!PlaceTheStack(params)) return false;
    LOG(INFO, "done placing stack\n");
    if (ShouldCancel()) return true;
  }
  return true;
}

bool ScoreActor::MoveStackIntoPosition(const ScoreParams& params) {
  // Move the fridge up a little bit.
  if (!SendGoal(0.0, params.upper_move_height, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    // Move on when it is clear of the tote knobs.
    if (CurrentGoalHeight() > params.begin_horizontal_move_height) {
      break;
    }
  }

  // Move the fridge out.
  if (!SendGoal(params.horizontal_move_target,
                params.begin_horizontal_move_height, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    if (NearGoal(params.horizontal_move_target,
                 params.begin_horizontal_move_height)) {
      LOG(INFO, "reached goal\n");
      break;
    }
  }

  if (ShouldCancel()) return true;

  return true;
}

bool ScoreActor::PlaceTheStack(const ScoreParams& params) {
  // Once the fridge is way out, put it on the ground.
  if (!SendGoal(params.horizontal_move_target, params.place_height, true)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    if (NearGoal(params.horizontal_move_target, params.place_height)) {
      break;
    }
  }

  if (ShouldCancel()) return true;

  // Release the grabbers.
  if (!SendGoal(params.horizontal_move_target, params.place_height, false)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  if (ShouldCancel()) return true;

  // Go back to the home position.
  if (!SendGoal(0.0, params.home_return_height, false)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      return true;
    }

    if (NearGoal(0.0, params.home_return_height)) {
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

double ScoreActor::CurrentGoalHeight() {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return 0.0;
  }

  ::aos::util::ElevatorArmKinematics::KinematicResult results;
  kinematics_.ForwardKinematic(fridge_queue.status->goal_height,
                               fridge_queue.status->goal_angle, 0.0, 0.0,
                               &results);

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
  new_fridge_goal->x = x;
  new_fridge_goal->y = y;
  new_fridge_goal->profiling_type = 1;
  new_fridge_goal->angular_velocity = 0.0;
  new_fridge_goal->grabbers.top_front = grabbers_enabled;
  new_fridge_goal->grabbers.top_back = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_front = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_back = grabbers_enabled;
  new_fridge_goal->max_x_velocity = kMaxXVelocity;
  new_fridge_goal->max_y_velocity = kMaxYVelocity;
  new_fridge_goal->max_x_acceleration = kMaxXAcceleration;
  new_fridge_goal->max_y_acceleration = kMaxYAcceleration;

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
