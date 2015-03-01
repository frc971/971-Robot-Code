#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/actors/horizontal_can_pickup_actor.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {
constexpr double kClawPickupVelocity = 3.00;
constexpr double kClawPickupAcceleration = 4.0;

constexpr double kArmVelocity = 1.00;
constexpr double kArmAcceleration = 1.6;
constexpr double kElevatorVelocity = 0.6;
constexpr double kElevatorAcceleration = 2.2;

constexpr double kAngleEpsilon = 0.10;

}  // namespace

HorizontalCanPickupActor::HorizontalCanPickupActor(
    HorizontalCanPickupActionQueueGroup *queues)
    : aos::common::actions::ActorBase<HorizontalCanPickupActionQueueGroup>(
          queues) {}

void HorizontalCanPickupActor::DoProfile(double height, double angle,
                                         bool grabbers) {
  DoProfile(height, angle, grabbers, grabbers, grabbers);
}

void HorizontalCanPickupActor::DoProfile(double height, double angle,
                                         bool top_grabbers, bool front_grabbers,
                                         bool back_grabbers) {
  FridgeProfileParams params;

  params.elevator_height = height;
  params.elevator_max_velocity = kElevatorVelocity;
  params.elevator_max_acceleration = kElevatorAcceleration;

  params.arm_angle = angle;
  params.arm_max_velocity = kArmVelocity;
  params.arm_max_acceleration = kArmAcceleration;

  params.top_front_grabber = top_grabbers;
  params.top_back_grabber = top_grabbers;
  params.bottom_front_grabber = front_grabbers;
  params.bottom_back_grabber = back_grabbers;

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

bool HorizontalCanPickupActor::WaitOrCancel(::aos::time::Time duration) {
  ::aos::time::Time end_time = ::aos::time::Time::Now() + duration;
  while (::aos::time::Time::Now() <= end_time) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) return false;
  }
  return true;
}

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
  DoProfile(params.elevator_height, 0.0, false, false, true);
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

  DoProfile(params.elevator_height, 0.0, false, true, true);

  MoveArm(params.claw_end_angle, 7.0);

  if (!WaitUntilNear(params.claw_end_angle)) {
    return true;
  }
  MoveArm(params.claw_end_angle, 0.0);

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
