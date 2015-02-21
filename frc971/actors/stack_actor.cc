#include <math.h>

#include "frc971/actors/stack_actor.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/constants.h"

namespace frc971 {
namespace actors {
namespace {

// TODO(danielp): Real numbers!
constexpr double kElevatorMaxVelocity = 0.5;
constexpr double kArmMaxVelocity = 0.5;
constexpr double kElevatorMaxAccel = 0.25;
constexpr double kArmMaxAccel = 0.25;

}  // namespace

StackActor::StackActor(StackActionQueueGroup* queues)
    : aos::common::actions::ActorBase<StackActionQueueGroup>(queues) {}

namespace {

void DoProfile(double height, bool grabbers) {
  FridgeProfileParams params;

  params.elevator_height = height;
  params.elevator_max_velocity = kElevatorMaxVelocity;
  params.elevator_max_acceleration = kElevatorMaxAccel;

  params.arm_angle = 0.0;
  params.arm_max_velocity = kArmMaxVelocity;
  params.arm_max_acceleration = kArmMaxAccel;

  params.top_front_grabber = grabbers;
  params.top_back_grabber = grabbers;
  params.bottom_front_grabber = grabbers;
  params.bottom_back_grabber = grabbers;

  ::std::unique_ptr<FridgeAction> profile = MakeFridgeProfileAction(params);
  profile->Start();
  profile->WaitUntilDone();
}

}  // namespace

bool StackActor::RunAction(const uint32_t&) {
  const auto& values = constants::GetValues();
  const double bottom = values.fridge.elevator.lower_limit;

  // Set the current stack down on top of the bottom box.
  DoProfile(bottom + values.tote_height, true);
  // Move down to enclose bottom box.
  DoProfile(bottom, false);
  // Clamp.
  DoProfile(bottom, true);

  return true;
}

::std::unique_ptr<StackAction> MakeStackAction() {
  return ::std::unique_ptr<StackAction>(
      new StackAction(&::frc971::actors::stack_action, 0));
}

}  // namespace actors
}  // namespace frc971
