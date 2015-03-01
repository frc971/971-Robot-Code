#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/actors/can_pickup_actor.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {

static constexpr double kArmVelocity = 1.00;
static constexpr double kArmAcceleration = 1.6;
static constexpr double kElevatorVelocity = 0.6;
static constexpr double kElevatorAcceleration = 2.2;

}  // namespace

// Elevator, arm
// Go to 0.3, 0.0
// Go to 0.35, -1.1

CanPickupActor::CanPickupActor(CanPickupActionQueueGroup *queues)
    : aos::common::actions::ActorBase<CanPickupActionQueueGroup>(queues) {}

void CanPickupActor::DoProfile(double height, double angle, bool grabbers) {
  FridgeProfileParams params;

  params.elevator_height = height;
  params.elevator_max_velocity = kElevatorVelocity;
  params.elevator_max_acceleration = kElevatorAcceleration;

  params.arm_angle = angle;
  params.arm_max_velocity = kArmVelocity;
  params.arm_max_acceleration = kArmAcceleration;

  params.top_front_grabber = grabbers;
  params.top_back_grabber = grabbers;
  params.bottom_front_grabber = grabbers;
  params.bottom_back_grabber = grabbers;

  ::std::unique_ptr<FridgeAction> profile = MakeFridgeProfileAction(params);
  profile->Start();
  while (!profile->CheckIteration()) {
    // wait until next Xms tick
    ::aos::time::PhasedLoopXMS(5, 2500);
    if (ShouldCancel()) {
      profile->Cancel();
      return;
    }
  }
}

bool CanPickupActor::RunAction(const CanPickupParams &params) {
  // Go around the can.
  DoProfile(params.pickup_height, params.pickup_angle, false);
  if (ShouldCancel()) return true;

  // Lift and grab.
  DoProfile(params.lift_height, params.pickup_angle, true);
  if (ShouldCancel()) return true;

  // Pull it back in.
  DoProfile(params.lift_height, params.end_angle, true);
  if (ShouldCancel()) return true;

  // Pull it back in.
  DoProfile(params.end_height, params.end_angle, true);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<CanPickupAction> MakeCanPickupAction(
    const CanPickupParams &params) {
  return ::std::unique_ptr<CanPickupAction>(
      new CanPickupAction(&::frc971::actors::can_pickup_action, params));
}

}  // namespace actors
}  // namespace frc971
