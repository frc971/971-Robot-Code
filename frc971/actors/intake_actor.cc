#include "frc971/actors/intake_actor.h"

#include <math.h>

#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "frc971/actors/claw_actor.h"
#include "frc971/constants.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/actors/stack_actor.h"

namespace frc971 {
namespace actors {
namespace {

using ::aos::time::Time;

// The angle the claw should be when intaking from the Human Player station.
// TODO(danielp): Make this right.
constexpr double kHpIntakeAngle = M_PI / 4.0;
// How long we spit backwards for when we have a tote and want to send it into
// the robot.
const Time kBackSpitTime = Time::InSeconds(0.5);

constexpr double kClawVelocity = 1.0;
constexpr double kClawIntakeVoltage = 12.0;
constexpr double kClawBackSpitVoltage = 12.0;
constexpr double kElevatorVelocity = 0.3;
constexpr double kElevatorAccel = 0.3;
constexpr double kArmVelocity = 0.3;
constexpr double kArmAccel = 0.3;

}  // namespace

IntakeActor::IntakeActor(IntakeActionQueueGroup *queues)
    : aos::common::actions::ActorBase<IntakeActionQueueGroup>(queues) {}

namespace {

// Creates and runs a claw action.
// angle: The angle we want to move the claw to.
// intake_voltage: The voltage to run the intake rollers at.
::std::unique_ptr<ClawAction> MoveClaw(double angle, double intake_voltage) {
  ClawParams params;
  params.claw_angle = angle;
  params.claw_max_velocity = kClawVelocity;
  params.intake_voltage = intake_voltage;
  params.rollers_closed = true;

  ::std::unique_ptr<ClawAction> claw_action = MakeClawAction(params);
  claw_action->Start();
  return claw_action;
}

// Creates and runs a fridge action.
// height: The height we want to move the elevator to.
::std::unique_ptr<FridgeAction> MoveFridge(double height) {
  FridgeProfileParams params;
  params.elevator_height = height;
  params.elevator_max_velocity = kElevatorVelocity;
  params.elevator_max_acceleration = kElevatorAccel;

  params.arm_angle = 0.0;
  params.arm_max_velocity = kArmVelocity;
  params.arm_max_acceleration = kArmAccel;

  params.top_front_grabber = true;
  params.top_back_grabber = true;
  params.bottom_front_grabber = true;
  params.bottom_back_grabber = true;

  ::std::unique_ptr<FridgeAction> fridge_action =
      MakeFridgeProfileAction(params);
  fridge_action->Start();
  return fridge_action;
}

// Put the systems into a reasonable configuration when we cancel.
void Reset() {
  const auto &values = constants::GetValues();

  auto claw = MoveClaw(kHpIntakeAngle, 0.0);
  auto fridge = MoveFridge(values.fridge.elevator.lower_limit);
  claw->WaitUntilDone();
  fridge->WaitUntilDone();
  LOG(INFO, "Done resetting systems.\n");
}

}  // namespace

bool IntakeActor::RunAction(const IntakeParams &params) {
  const auto &values = constants::GetValues();

  if (params.grab) {
    // Grab the tote.
    LOG(INFO, "Grabbing tote.\n");

    // Move the fridge up to make space for the tote we're intaking.
    auto fridge = MoveFridge(values.tote_height);
    // Start intaking.
    const double claw_angle = params.ground ? 0.0 : kClawIntakeVoltage;
    auto claw = MoveClaw(claw_angle, kClawIntakeVoltage);
    fridge->WaitUntilDone();
    claw->WaitUntilDone();
  } else {
    // Send it back into the robot.
    LOG(INFO, "Spitting tote backwards.\n");

    // Pull the claw up.
    MoveClaw(kHpIntakeAngle, 0.0);
    if (ShouldCancel()) {
      LOG(INFO, "Cancelling.\n");
      Reset();
      return true;
    }

    // Spit backwards for awhile.
    MoveClaw(kHpIntakeAngle, kClawBackSpitVoltage);
    ::aos::time::SleepFor(kBackSpitTime);
    MoveClaw(kHpIntakeAngle, 0.0);

    // Stack the new tote.
    StackParams params;
    params.claw_out_angle = M_PI / 4;
    ::std::unique_ptr<StackAction> stack_action = MakeStackAction(params);
    stack_action->Start();
    stack_action->WaitUntilDone();
  }

  LOG(INFO, "Done running intake action.\n");
  return true;
}

::std::unique_ptr<IntakeAction> MakeIntakeAction(const IntakeParams &params) {
  return ::std::unique_ptr<IntakeAction>(
      new IntakeAction(&::frc971::actors::intake_action, params));
}

}  // actors
}  // frc971
