#include "frc971/actors/stack_and_hold_actor.h"

#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {
constexpr ProfileParams kSlowArmMove{0.8, 1.4};
constexpr ProfileParams kSlowElevatorMove{0.5, 3.0};
constexpr ProfileParams kReallySlowElevatorMove{0.10, 1.0};

constexpr ProfileParams kFastArmMove{0.8, 4.0};
constexpr ProfileParams kFastElevatorMove{1.2, 5.0};
}  // namespace

StackAndHoldActor::StackAndHoldActor(StackAndHoldActionQueueGroup *queues)
    : FridgeActorBase<StackAndHoldActionQueueGroup>(queues) {}

bool StackAndHoldActor::RunAction(const StackAndHoldParams &params) {
  const auto &values = constants::GetValues();

  // Set the current stack down on top of the bottom box.
  DoFridgeProfile(params.over_box_before_place_height, 0.0, kSlowArmMove,
                  kReallySlowElevatorMove, true);
  if (ShouldCancel()) return true;
  // Set down on the box.
  DoFridgeProfile(params.bottom + values.tote_height, 0.0, kSlowArmMove,
                  kSlowElevatorMove, true);
  if (ShouldCancel()) return true;

  // Release
  DoFridgeProfile(params.bottom + values.tote_height, 0.0, kFastArmMove,
                  kFastElevatorMove, false);
  if (ShouldCancel()) return true;

  if (!WaitOrCancel(aos::time::Time::InSeconds(params.clamp_pause_time))) {
    return true;
  }

  // Go up.
  DoFridgeProfile(params.hold_height, params.arm_clearance, kFastArmMove,
                  kFastElevatorMove, false);
  if (ShouldCancel()) return true;
  // Move back
  DoFridgeProfile(params.hold_height, 0.0, kFastArmMove, kFastElevatorMove,
                  false);
  if (ShouldCancel()) return true;
  // Grab
  DoFridgeProfile(params.hold_height, 0.0, kFastArmMove, kFastElevatorMove,
                  true);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<StackAndHoldAction> MakeStackAndHoldAction(
    const StackAndHoldParams &params) {
  return ::std::unique_ptr<StackAndHoldAction>(
      new StackAndHoldAction(&::frc971::actors::stack_and_hold_action, params));
}

}  // namespace actors
}  // namespace frc971
