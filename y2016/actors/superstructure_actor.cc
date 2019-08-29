#include "y2016/actors/superstructure_actor.h"

#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"
#include "y2016/actors/superstructure_actor.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"

namespace y2016 {
namespace actors {

namespace chrono = ::std::chrono;

SuperstructureActor::SuperstructureActor(::aos::EventLoop *event_loop)
    : aos::common::actions::ActorBase<superstructure_action::Goal>(
          event_loop, "/superstructure_action"),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2016::control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2016::control_loops::superstructure::Status>(
                  "/superstructure")) {}

bool SuperstructureActor::RunAction(
    const superstructure_action::SuperstructureActionParams *params) {
  AOS_LOG(INFO, "Starting superstructure action\n");

  MoveSuperstructure(params->partial_angle(), params->shooter_angle(), false);
  WaitForSuperstructure();
  if (ShouldCancel()) return true;
  MoveSuperstructure(params->partial_angle(), params->shooter_angle(), true);
  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params->delay_time()))))
    return true;
  MoveSuperstructure(params->full_angle(), params->shooter_angle(), true);
  WaitForSuperstructure();
  if (ShouldCancel()) return true;
  return true;
}

void SuperstructureActor::MoveSuperstructure(double shoulder, double shooter,
                                             bool unfold_climber) {
  superstructure_goal_ = {0, shoulder, shooter};

  auto builder = superstructure_goal_sender_.MakeBuilder();

  control_loops::superstructure::Goal::Builder superstructure_goal_builder =
      builder.MakeBuilder<control_loops::superstructure::Goal>();

  superstructure_goal_builder.add_angle_intake(0);
  superstructure_goal_builder.add_angle_shoulder(shoulder);
  superstructure_goal_builder.add_angle_wrist(shooter);

  superstructure_goal_builder.add_max_angular_velocity_intake(7.0);
  superstructure_goal_builder.add_max_angular_velocity_shoulder(4.0);
  superstructure_goal_builder.add_max_angular_velocity_wrist(10.0);

  superstructure_goal_builder.add_max_angular_acceleration_intake(40.0);
  superstructure_goal_builder.add_max_angular_acceleration_shoulder(5.0);
  superstructure_goal_builder.add_max_angular_acceleration_wrist(25.0);

  superstructure_goal_builder.add_voltage_top_rollers(0);
  superstructure_goal_builder.add_voltage_bottom_rollers(0);

  superstructure_goal_builder.add_traverse_unlatched(true);
  superstructure_goal_builder.add_traverse_down(false);
  superstructure_goal_builder.add_voltage_climber(0.0);
  superstructure_goal_builder.add_unfold_climber(unfold_climber);

  if (!builder.Send(superstructure_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending superstructure move failed.\n");
  }
}

bool SuperstructureActor::SuperstructureProfileDone() {
  constexpr double kProfileError = 1e-2;
  return ::std::abs(superstructure_status_fetcher_->intake()->goal_angle() -
                    superstructure_goal_.intake) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->shoulder()->goal_angle() -
                    superstructure_goal_.shoulder) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->intake()
                        ->goal_angular_velocity()) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->shoulder()
                        ->goal_angular_velocity()) < kProfileError;
}

bool SuperstructureActor::SuperstructureDone() {
  superstructure_status_fetcher_.Fetch();

  // We are no longer running if we are in the zeroing states (below 12), or
  // estopped.
  if (superstructure_status_fetcher_->state() < 12 ||
      superstructure_status_fetcher_->state() == 16) {
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
