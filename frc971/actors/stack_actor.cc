#include "frc971/actors/stack_actor.h"

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

StackActor::StackActor(StackActionQueueGroup *queues)
    : FridgeActorBase<StackActionQueueGroup>(queues) {}

bool StackActor::RunAction(const StackParams &params) {
  const auto &values = constants::GetValues();
  const double bottom = 0.020;

  // Set the current stack down on top of the bottom box.
  DoFridgeProfile(0.39, 0.0, kSlowArmMove, kReallySlowElevatorMove, true);
  if (ShouldCancel()) return true;
  // Set down on the box.
  DoFridgeProfile(bottom + values.tote_height, 0.0, kSlowArmMove,
                  kSlowElevatorMove, true);
  if (ShouldCancel()) return true;
  // Clamp.
  {
    auto message = control_loops::claw_queue.goal.MakeMessage();
    message->angle = params.claw_out_angle;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }
  DoFridgeProfile(bottom, -0.05, kFastArmMove, kFastElevatorMove, false);
  if (ShouldCancel()) return true;
  DoFridgeProfile(bottom, 0.0, kFastArmMove, kFastElevatorMove, false);
  if (ShouldCancel()) return true;
  aos::time::SleepFor(aos::time::Time::InMS(100));

  return true;
}

::std::unique_ptr<StackAction> MakeStackAction(const StackParams &params) {
  return ::std::unique_ptr<StackAction>(
      new StackAction(&::frc971::actors::stack_action, params));
}

}  // namespace actors
}  // namespace frc971
