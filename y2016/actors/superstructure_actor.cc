#include "y2016/actors/superstructure_actor.h"

#include <cmath>

#include "aos/util/phased_loop.h"
#include "aos/logging/logging.h"
#include "y2016/actors/superstructure_actor.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

namespace y2016 {
namespace actors {

namespace chrono = ::std::chrono;

SuperstructureActor::SuperstructureActor(::aos::EventLoop *event_loop)
    : aos::common::actions::ActorBase<actors::SuperstructureActionQueueGroup>(
          event_loop, ".y2016.actors.superstructure_action"),
      superstructure_goal_sender_(
          event_loop
              ->MakeSender<::y2016::control_loops::SuperstructureQueue::Goal>(
                  ".y2016.control_loops.superstructure_queue.goal")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<
              ::y2016::control_loops::SuperstructureQueue::Status>(
              ".y2016.control_loops.superstructure_queue.status")) {}

bool SuperstructureActor::RunAction(
    const actors::SuperstructureActionParams &params) {
  AOS_LOG(INFO, "Starting superstructure action\n");

  MoveSuperstructure(params.partial_angle, params.shooter_angle, false);
  WaitForSuperstructure();
  if (ShouldCancel()) return true;
  MoveSuperstructure(params.partial_angle, params.shooter_angle, true);
  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.delay_time))))
    return true;
  MoveSuperstructure(params.full_angle, params.shooter_angle, true);
  WaitForSuperstructure();
  if (ShouldCancel()) return true;
  return true;
}

void SuperstructureActor::MoveSuperstructure(double shoulder, double shooter,
                                             bool unfold_climber) {
  superstructure_goal_ = {0, shoulder, shooter};

  auto new_superstructure_goal = superstructure_goal_sender_.MakeMessage();

  new_superstructure_goal->angle_intake = 0;
  new_superstructure_goal->angle_shoulder = shoulder;
  new_superstructure_goal->angle_wrist = shooter;

  new_superstructure_goal->max_angular_velocity_intake = 7.0;
  new_superstructure_goal->max_angular_velocity_shoulder = 4.0;
  new_superstructure_goal->max_angular_velocity_wrist = 10.0;

  new_superstructure_goal->max_angular_acceleration_intake = 40.0;
  new_superstructure_goal->max_angular_acceleration_shoulder = 5.0;
  new_superstructure_goal->max_angular_acceleration_wrist = 25.0;

  new_superstructure_goal->voltage_top_rollers = 0;
  new_superstructure_goal->voltage_bottom_rollers = 0;

  new_superstructure_goal->traverse_unlatched = true;
  new_superstructure_goal->traverse_down = false;
  new_superstructure_goal->voltage_climber = 0.0;
  new_superstructure_goal->unfold_climber = unfold_climber;

  if (!new_superstructure_goal.Send()) {
    AOS_LOG(ERROR, "Sending superstructure move failed.\n");
  }
}

bool SuperstructureActor::SuperstructureProfileDone() {
  constexpr double kProfileError = 1e-2;
  return ::std::abs(superstructure_status_fetcher_->intake.goal_angle -
                    superstructure_goal_.intake) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->shoulder.goal_angle -
                    superstructure_goal_.shoulder) < kProfileError &&
         ::std::abs(
             superstructure_status_fetcher_->intake.goal_angular_velocity) <
             kProfileError &&
         ::std::abs(
             superstructure_status_fetcher_->shoulder.goal_angular_velocity) <
             kProfileError;
}

bool SuperstructureActor::SuperstructureDone() {
  superstructure_status_fetcher_.Fetch();

  // We are no longer running if we are in the zeroing states (below 12), or
  // estopped.
  if (superstructure_status_fetcher_->state < 12 ||
      superstructure_status_fetcher_->state == 16) {
    AOS_LOG(ERROR, "Superstructure no longer running, aborting action\n");
    return true;
  }

  if (SuperstructureProfileDone()) {
    AOS_LOG(DEBUG, "Profile done.\n");
    return true;
  }
  return false;
}

void SuperstructureActor::WaitForSuperstructure() {
  WaitUntil(::std::bind(&SuperstructureActor::SuperstructureDone, this));
}

}  // namespace actors
}  // namespace y2016
