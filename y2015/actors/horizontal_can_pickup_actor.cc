#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "y2015/actors/horizontal_can_pickup_actor.h"
#include "y2015/actors/fridge_profile_lib.h"
#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw.q.h"

namespace y2015 {
namespace actors {
namespace {
constexpr ProfileParams kClawPickup{3.0, 2.0};
constexpr ProfileParams kClawBackDown{7.0, 10.0};
constexpr ProfileParams kClawInitialLift{7.0, 8.0};

constexpr ProfileParams kArmMove{1.0, 1.6};
constexpr ProfileParams kElevatorMove{0.6, 2.2};

constexpr ProfileParams kFastArmMove{2.0, 3.0};
constexpr ProfileParams kFastElevatorMove{1.0, 3.0};

constexpr double kAngleEpsilon = 0.10;
constexpr double kGoalAngleEpsilon = 0.01;

}  // namespace

namespace chrono = ::std::chrono;
using ::y2015::control_loops::claw_queue;

HorizontalCanPickupActor::HorizontalCanPickupActor(
    HorizontalCanPickupActionQueueGroup *queues)
    : FridgeActorBase<HorizontalCanPickupActionQueueGroup>(queues) {}

bool HorizontalCanPickupActor::WaitUntilGoalNear(double angle) {
  while (true) {
    claw_queue.status.FetchAnother();
    if (ShouldCancel()) return false;
    const double goal_angle = claw_queue.status->goal_angle;
    LOG_STRUCT(DEBUG, "Got claw status", *claw_queue.status);

    if (::std::abs(goal_angle - angle) < kGoalAngleEpsilon) {
      return true;
    }
  }
}

bool HorizontalCanPickupActor::WaitUntilNear(double angle) {
  while (true) {
    claw_queue.status.FetchAnother();
    if (ShouldCancel()) return false;
    const double current_angle = claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *claw_queue.status);

    if (::std::abs(current_angle - angle) < kAngleEpsilon) {
      return true;
    }
  }
}
void HorizontalCanPickupActor::MoveArm(double angle, double intake_power) {
  MoveArm(angle, intake_power, kClawPickup);
}

void HorizontalCanPickupActor::MoveArm(double angle, double intake_power,
                                       const ProfileParams profile_params) {
  auto message = claw_queue.goal.MakeMessage();
  message->angle = angle;
  message->max_velocity = profile_params.velocity;
  message->max_acceleration = profile_params.acceleration;
  message->angular_velocity = 0.0;
  message->intake = intake_power;
  message->rollers_closed = true;

  LOG_STRUCT(DEBUG, "Sending claw goal", *message);
  message.Send();
}

bool HorizontalCanPickupActor::RunAction(
    const HorizontalCanPickupParams &params) {
  // Go around the can.
  if (!StartFridgeProfile(params.elevator_height, 0.0, kFastElevatorMove,
                          kFastArmMove, false, false, true)) {
    return true;
  }

  claw_queue.status.FetchAnother();

  MoveArm(claw_queue.status->angle, params.spit_power);

  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.spit_time)))) {
    return true;
  }

  MoveArm(params.pickup_angle, 0.0, kClawInitialLift);

  if (!WaitUntilNear(params.pickup_angle)) {
    return true;
  }

  MoveArm(params.pickup_angle, params.suck_power);

  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.suck_time)))) {
    return true;
  }

  MoveArm(0.0, 0.0, kClawBackDown);

  if (!WaitUntilGoalNear(0.0)) {
    return true;
  }

  MoveArm(0.0, params.claw_settle_power);

  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.claw_settle_time)))) {
    return true;
  }

  while (true) {
    ProfileStatus status =
        IterateProfile(params.elevator_height, 0.0, kFastElevatorMove,
                       kFastArmMove, false, false, true);
    if (status == DONE) {
      break;
    } else if (status == CANCELED) {
      return true;
    }
  }

  MoveArm(params.claw_full_lift_angle, 0.0);

  if (!WaitUntilNear(params.claw_full_lift_angle)) {
    return true;
  }

  DoFridgeProfile(params.elevator_height, 0.0, kElevatorMove, kArmMove, false,
                  true, true);
  if (ShouldCancel()) return true;

  MoveArm(params.claw_end_angle, 7.0);

  if (!WaitUntilNear(params.claw_end_angle)) {
    return true;
  }
  MoveArm(params.claw_end_angle, 0.0);

  if (ShouldCancel()) return true;

  DoFridgeProfile(params.elevator_end_height, params.arm_end_angle,
                  kElevatorMove, kArmMove, false, true, true);

  return true;
}

::std::unique_ptr<HorizontalCanPickupAction> MakeHorizontalCanPickupAction(
    const HorizontalCanPickupParams &params) {
  return ::std::unique_ptr<HorizontalCanPickupAction>(
      new HorizontalCanPickupAction(
          &::y2015::actors::horizontal_can_pickup_action, params));
}

}  // namespace actors
}  // namespace y2015
