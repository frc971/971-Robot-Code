#include "frc971/actors/score_actor.h"

#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/phased_loop.h"
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

ScoreActor::ScoreActor(ScoreActionQueueGroup* queues)
    : aos::common::actions::ActorBase<ScoreActionQueueGroup>(queues) {}

void ScoreActor::DoProfile(double height, double angle, bool grabbers) {
  FridgeProfileParams params;

  params.elevator_height = height;
  params.elevator_max_velocity = kElevatorMaxVelocity;
  params.elevator_max_acceleration = kElevatorMaxAccel;

  params.arm_angle = angle;
  params.arm_max_velocity = kArmMaxVelocity;
  params.arm_max_acceleration = kArmMaxAccel;

  params.top_front_grabber = grabbers;
  params.top_back_grabber = grabbers;
  params.bottom_front_grabber = grabbers;
  params.bottom_back_grabber = grabbers;

  ::std::unique_ptr<FridgeAction> profile = MakeFridgeProfileAction(params);
  profile->Start();
  while (!profile->CheckIteration()) {
    ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(), 2500);
    if (ShouldCancel()) {
      LOG(WARNING, "Cancelling profile.\n");
      profile->Cancel();
      return;
    }
  }
}

bool ScoreActor::RunAction(const ScoreParams& params) {
  const auto& values = constants::GetValues();

  // We're going to move the elevator first so we don't crash the fridge into
  // the ground.
  DoProfile(values.fridge.arm_zeroing_height, 0.0, true);
  if (ShouldCancel()) return true;
  // Now move them both together.
  DoProfile(params.height, M_PI / 2.0, true);
  if (ShouldCancel()) return true;
  // Release the totes.
  DoProfile(values.fridge.arm_zeroing_height, 0.0, false);
  if (ShouldCancel()) return true;
  // Retract. Move back to our lowered position.
  DoProfile(values.fridge.elevator.lower_limit, 0.0, false);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<ScoreAction> MakeScoreAction(const ScoreParams& params) {
  return ::std::unique_ptr<ScoreAction>(
      new ScoreAction(&::frc971::actors::score_action, params));
}

}  // namespace actors
}  // namespace frc971
