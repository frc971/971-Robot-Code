#include <math.h>

#include "aos/common/time.h"
#include "frc971/actors/lift_actor.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {

static constexpr double kArmVelocity = 0.40;
static constexpr double kArmAcceleration = 1.0;
static constexpr double kElevatorVelocity = 0.5;
static constexpr double kElevatorAcceleration = 2.2;

}  // namespace

LiftActor::LiftActor(LiftActionQueueGroup *queues)
    : aos::common::actions::ActorBase<LiftActionQueueGroup>(queues) {}

namespace {

void DoProfile(double height, double angle, bool grabbers) {
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
  profile->WaitUntilDone();
}

}  // namespace

bool LiftActor::RunAction(const LiftParams &params) {
  // Lift the box straight up.
  DoProfile(params.lift_height, 0.0, true);
  // Move it back to the storage location.
  DoProfile(params.lift_height, params.lift_arm, true);

  return true;
}

::std::unique_ptr<LiftAction> MakeLiftAction(const LiftParams &params) {
  return ::std::unique_ptr<LiftAction>(
      new LiftAction(&::frc971::actors::lift_action, params));
}

}  // namespace actors
}  // namespace frc971
