#include "frc971/actors/score_actor.h"

#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/phased_loop.h"
#include "frc971/actors/fridge_profile_lib.h"
#include "frc971/constants.h"

namespace frc971 {
namespace actors {

namespace {
constexpr ProfileParams kElevatorMove{0.5, 2.0};
constexpr ProfileParams kArmMove{0.5, 1.0};
}  // namespace

ScoreActor::ScoreActor(ScoreActionQueueGroup* queues)
    : FridgeActorBase<ScoreActionQueueGroup>(queues) {}

bool ScoreActor::RunAction(const ScoreParams& params) {
  const auto& values = constants::GetValues();

  // We're going to move the elevator first so we don't crash the fridge into
  // the ground.
  DoFridgeProfile(values.fridge.arm_zeroing_height, 0.0, kElevatorMove,
                  kArmMove, true);
  if (ShouldCancel()) return true;
  // Now move them both together.
  DoFridgeProfile(params.height, M_PI / 2.0, kElevatorMove, kArmMove, true);
  if (ShouldCancel()) return true;
  // Release the totes.
  DoFridgeProfile(values.fridge.arm_zeroing_height, 0.0, kElevatorMove,
                  kArmMove, false);
  if (ShouldCancel()) return true;
  // Retract. Move back to our lowered position.
  DoFridgeProfile(values.fridge.elevator.lower_limit, 0.0, kElevatorMove,
                  kArmMove, false);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams& params) {
  return ::std::unique_ptr<ScoreAction>(
      new ScoreAction(&::frc971::actors::score_action, params));
}

}  // namespace actors
}  // namespace frc971
