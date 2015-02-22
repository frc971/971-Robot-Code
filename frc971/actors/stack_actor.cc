#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/actors/stack_actor.h"
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

StackActor::StackActor(StackActionQueueGroup *queues)
    : aos::common::actions::ActorBase<StackActionQueueGroup>(queues) {}

void StackActor::DoProfile(double height, double angle, bool grabbers) {
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

bool StackActor::RunAction(const StackParams &params) {
  const auto &values = constants::GetValues();
  const double bottom = 0.020;

  // Set the current stack down on top of the bottom box.
  DoProfile(0.45, 0.0, true);
  if (ShouldCancel()) return true;
  // Move down to enclose bottom box.
  DoProfile(bottom + values.tote_height, 0.0, true);
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
  DoProfile(bottom, -0.05, false);
  DoProfile(bottom, 0.0, false);
  aos::time::SleepFor(aos::time::Time::InMS(100));

  return true;
}

::std::unique_ptr<StackAction> MakeStackAction(const StackParams &params) {
  return ::std::unique_ptr<StackAction>(
      new StackAction(&::frc971::actors::stack_action, params));
}

}  // namespace actors
}  // namespace frc971
