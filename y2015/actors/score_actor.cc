#include "y2015/actors/score_actor.h"

#include <cmath>

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/queue_logging.h"

#include "y2015/constants.h"
#include "y2015/control_loops/fridge/fridge.q.h"

using ::y2015::control_loops::fridge::fridge_queue;

namespace chrono = ::std::chrono;

namespace y2015 {
namespace actors {
namespace {

const double kSlowMaxXVelocity = 0.80;
const double kSlowMaxYVelocity = 0.40;
const double kFastMaxXVelocity = 0.80;
const double kFastMaxYVelocity = 0.30;
const double kReallyFastMaxXVelocity = 1.0;
const double kReallyFastMaxYVelocity = 0.6;

const double kMaxXAcceleration = 0.5;
const double kMaxYAcceleration = 0.7;
const double kLiftYAcceleration = 2.0;
const double kLowerYAcceleration = 2.0;
const double kFastMaxXAcceleration = 1.5;
const double kFastMaxYAcceleration = 1.5;
const double kSlowMaxXAcceleration = 0.3;
const double kSlowMaxYAcceleration = 0.5;

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
  if (!SendGoal(0.0, params.upper_move_height, true, kSlowMaxXVelocity,
                kSlowMaxYVelocity, kMaxXAcceleration, kLiftYAcceleration)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(chrono::duration_cast<chrono::milliseconds>(
                                   ::aos::controls::kLoopFrequency).count(),
                               2500);
    if (ShouldCancel()) {
      return true;
    }

    // Move on when it is clear of the tote knobs.
    if (CurrentGoalHeight() > params.begin_horizontal_move_height) {
      LOG(INFO, "moving on to horizontal move\n");
      break;
    }
  }

  // Move the fridge out.
  if (!SendGoal(params.horizontal_move_target,
                params.begin_horizontal_move_height, true, kSlowMaxXVelocity,
                kFastMaxYVelocity, kSlowMaxXAcceleration,
                kSlowMaxYAcceleration)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  bool started_lowering = false;

  while (true) {
    ::aos::time::PhasedLoopXMS(chrono::duration_cast<chrono::milliseconds>(
                                   ::aos::controls::kLoopFrequency).count(),
                               2500);
    if (ShouldCancel()) {
      return true;
    }
    // Round the moving out corner and start setting down.
    if (params.place_the_stack && !started_lowering) {
      if (CurrentGoalX() < params.horizontal_start_lowering) {
        if (!SendGoal(params.horizontal_move_target, params.place_height, true,
                      kSlowMaxXVelocity, kFastMaxYVelocity,
                      kSlowMaxXAcceleration, kMaxYAcceleration)) {
          LOG(ERROR, "Sending fridge message failed.\n");
          return false;
        }
        started_lowering = true;
      }
    }

    if (NearHorizontalGoal(params.horizontal_move_target)) {
      LOG(INFO, "reached goal\n");
      break;
    }
  }

  if (ShouldCancel()) return true;

  return true;
}

bool ScoreActor::PlaceTheStack(const ScoreParams& params) {
  // Once the fridge is way out, put it on the ground.
  if (!SendGoal(params.horizontal_move_target, params.place_height, true,
                kFastMaxXVelocity, kFastMaxYVelocity, kMaxXAcceleration,
                kLowerYAcceleration)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  while (true) {
    ::aos::time::PhasedLoopXMS(chrono::duration_cast<chrono::milliseconds>(
                                   ::aos::controls::kLoopFrequency).count(),
                               2500);
    if (ShouldCancel()) {
      return true;
    }

    if (NearGoal(params.horizontal_move_target, params.place_height)) {
      break;
    }
  }

  // Release and give the grabers a chance to get out of the way.
  if (!SendGoal(params.horizontal_move_target, params.place_height, false,
                kFastMaxXVelocity, kFastMaxYVelocity, kMaxXAcceleration,
                kLowerYAcceleration)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }
  if (!WaitOrCancel(chrono::milliseconds(100))) return true;

  // Go back to the home position.
  if (!SendGoal(0.0, params.place_height, false, kReallyFastMaxXVelocity,
                kReallyFastMaxYVelocity, kFastMaxXAcceleration,
                kFastMaxYAcceleration)) {
    LOG(ERROR, "Sending fridge message failed.\n");
    return false;
  }

  bool has_lifted = false;
  while (true) {
    ::aos::time::PhasedLoopXMS(chrono::duration_cast<chrono::milliseconds>(
                                   ::aos::controls::kLoopFrequency).count(),
                               2500);
    if (ShouldCancel()) {
      return true;
    }

    if (!has_lifted &&
        CurrentGoalX() > params.home_lift_horizontal_start_position) {
      if (!SendGoal(0.0, params.home_return_height, false,
                    kReallyFastMaxXVelocity, kReallyFastMaxYVelocity,
                    kFastMaxXAcceleration, kFastMaxYAcceleration)) {
        LOG(ERROR, "Sending fridge message failed.\n");
        return false;
      }
      has_lifted = true;
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

  return fridge_queue.status->x;
}

double ScoreActor::CurrentGoalX() {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return 0.0;
  }

  return fridge_queue.status->goal_x;
}

bool ScoreActor::SendGoal(double x, double y, bool grabbers_enabled,
                          double max_x_velocity, double max_y_velocity,
                          double max_x_acceleration,
                          double max_y_acceleration) {
  auto new_fridge_goal = fridge_queue.goal.MakeMessage();
  new_fridge_goal->x = x;
  new_fridge_goal->y = y;
  new_fridge_goal->profiling_type = 1;
  new_fridge_goal->angular_velocity = 0.0;
  new_fridge_goal->velocity = 0.0;
  new_fridge_goal->grabbers.top_front = grabbers_enabled;
  new_fridge_goal->grabbers.top_back = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_front = grabbers_enabled;
  new_fridge_goal->grabbers.bottom_back = grabbers_enabled;
  new_fridge_goal->max_x_velocity = max_x_velocity;
  new_fridge_goal->max_y_velocity = max_y_velocity;
  new_fridge_goal->max_x_acceleration = max_x_acceleration;
  new_fridge_goal->max_y_acceleration = max_y_acceleration;
  LOG_STRUCT(INFO, "sending fridge goal", *new_fridge_goal);

  return new_fridge_goal.Send();
}

bool ScoreActor::NearHorizontalGoal(double x) {
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
          ::std::abs(goal_results.fridge_x - x) < 0.0001);
}

bool ScoreActor::NearGoal(double x, double y) {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return false;
  }

  const auto &status = *fridge_queue.status;
  return (::std::abs(status.x - x) < 0.020 &&
          ::std::abs(status.y - y) < 0.020 &&
          ::std::abs(status.goal_x - x) < 0.0001 &&
          ::std::abs(status.goal_y - y) < 0.0001);
}

::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams& params) {
  return ::std::unique_ptr<ScoreAction>(
      new ScoreAction(&::y2015::actors::score_action, params));
}

}  // namespace actors
}  // namespace y2015
