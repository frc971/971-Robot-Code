#include "y2015/actors/pickup_actor.h"

#include <chrono>
#include <cmath>

#include "aos/common/controls/control_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "y2015/control_loops/claw/claw.q.h"

namespace y2015 {
namespace actors {
namespace {
constexpr double kClawPickupVelocity = 3.00;
constexpr double kClawPickupAcceleration = 3.5;
constexpr double kClawMoveDownVelocity = 7.00;
constexpr double kClawMoveDownAcceleration = 15.0;
constexpr double kClawMoveUpVelocity = 8.0;
constexpr double kClawMoveUpAcceleration = 25.0;
}  // namespace

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
using ::y2015::control_loops::claw_queue;

PickupActor::PickupActor(PickupActionQueueGroup* queues)
    : aos::common::actions::ActorBase<PickupActionQueueGroup>(queues) {}

bool PickupActor::RunAction(const PickupParams& params) {
  constexpr double kAngleEpsilon = 0.10;
  // Start lifting the tote.
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.pickup_angle;
    message->max_velocity = kClawPickupVelocity;
    message->max_acceleration = kClawPickupAcceleration;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }
  while (true) {
    claw_queue.status.FetchAnother();
    if (ShouldCancel()) return true;
    const double current_angle = claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *claw_queue.status);

    if (current_angle > params.suck_angle ||
        ::std::abs(current_angle - params.pickup_angle) < kAngleEpsilon) {
      break;
    }
  }

  // Once above params.suck_angle, start sucking while lifting.
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.pickup_angle;
    message->max_velocity = kClawPickupVelocity;
    message->max_acceleration = kClawPickupAcceleration;
    message->angular_velocity = 0.0;
    message->intake = params.intake_voltage;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }

  while (true) {
    claw_queue.status.FetchAnother();
    if (ShouldCancel()) return true;
    const double current_angle = claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *claw_queue.status);

    if (::std::abs(current_angle - params.pickup_angle) < kAngleEpsilon) {
      break;
    }
  }

  // Now that we have reached the upper height, come back down while intaking.
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.suck_angle_finish;
    message->max_velocity = kClawMoveDownVelocity;
    message->max_acceleration = kClawMoveDownAcceleration;
    message->angular_velocity = 0.0;
    message->intake = params.intake_voltage;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }

  // Pull in for params.intake_time.
  monotonic_clock::time_point end_time =
      monotonic_clock::now() +
      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(params.intake_time));
  ::aos::time::PhasedLoop phased_loop(::aos::controls::kLoopFrequency,
                                      ::std::chrono::milliseconds(5) / 2);
  while (monotonic_clock::now() <= end_time) {
    phased_loop.SleepUntilNext();
    if (ShouldCancel()) return true;
  }

  // Lift the claw back up to pack the box back in.
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.pickup_finish_angle;
    message->max_velocity = kClawMoveUpVelocity;
    message->max_acceleration = kClawMoveUpAcceleration;
    message->angular_velocity = 0.0;
    message->intake = params.intake_voltage;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }

  while (true) {
    claw_queue.status.FetchAnother();
    if (ShouldCancel()) return true;
    const double current_angle = claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *claw_queue.status);

    if (::std::abs(current_angle - params.pickup_finish_angle) <
        kAngleEpsilon) {
      break;
    }
  }

  // Stop the motors...
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.pickup_finish_angle;
    message->max_velocity = kClawMoveUpVelocity;
    message->max_acceleration = kClawMoveUpAcceleration;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = true;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }


  return true;
}

::std::unique_ptr<PickupAction> MakePickupAction(const PickupParams& params) {
  return ::std::unique_ptr<PickupAction>(
      new PickupAction(&::y2015::actors::pickup_action, params));
}

}  // namespace actors
}  // namespace y2015
