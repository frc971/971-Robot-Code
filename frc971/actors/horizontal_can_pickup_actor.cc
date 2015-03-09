#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/actors/horizontal_can_pickup_actor.h"
#include "frc971/actors/fridge_profile_lib.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {
constexpr double kClawPickupVelocity = 3.00;
constexpr double kClawPickupAcceleration = 4.0;

constexpr ProfileParams kArmMove{1.0, 1.6};
constexpr ProfileParams kElevatorMove{0.6, 2.2};

constexpr double kAngleEpsilon = 0.10;

}  // namespace

HorizontalCanPickupActor::HorizontalCanPickupActor(
    HorizontalCanPickupActionQueueGroup *queues)
    : FridgeActorBase<HorizontalCanPickupActionQueueGroup>(queues) {}

bool HorizontalCanPickupActor::WaitUntilNear(double angle) {
  while (true) {
    control_loops::claw_queue.status.FetchAnother();
    if (ShouldCancel()) return false;
    const double current_angle = control_loops::claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *control_loops::claw_queue.status);

    if (::std::abs(current_angle - angle) < kAngleEpsilon) {
      return true;
    }
  }
}

void HorizontalCanPickupActor::MoveArm(double angle, double intake_power) {
  {
    auto message = control_loops::claw_queue.goal.MakeMessage();
    message->angle = angle;
    message->max_velocity = kClawPickupVelocity;
    message->max_acceleration = kClawPickupAcceleration;
    message->angular_velocity = 0.0;
    message->intake = intake_power;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }
}

bool HorizontalCanPickupActor::RunAction(
    const HorizontalCanPickupParams &params) {
  // Go around the can.
  DoFridgeProfile(params.elevator_height, 0.0, kElevatorMove, kArmMove, false,
                  false, true);
  if (ShouldCancel()) return true;

  MoveArm(params.pickup_angle, 0.0);

  if (!WaitUntilNear(params.pickup_angle)) {
    return true;
  }

  MoveArm(params.pickup_angle, params.suck_power);

  if (!WaitOrCancel(aos::time::Time::InSeconds(params.suck_time))) {
    return true;
  }

  MoveArm(0.0, 0.0);

  if (!WaitUntilNear(0.0)) {
    return true;
  }

  MoveArm(0.0, params.claw_settle_power);

  if (!WaitOrCancel(aos::time::Time::InSeconds(params.claw_settle_time))) {
    return true;
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
          &::frc971::actors::horizontal_can_pickup_action, params));
}

}  // namespace actors
}  // namespace frc971
