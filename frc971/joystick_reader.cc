#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/actions/actions.h"

#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/actors/pickup_actor.h"
#include "frc971/actors/stack_actor.h"
#include "frc971/actors/score_actor.h"
#include "frc971/actors/stack_and_lift_actor.h"
#include "frc971/actors/stack_and_hold_actor.h"
#include "frc971/actors/held_to_lift_actor.h"
#include "frc971/actors/lift_actor.h"
#include "frc971/actors/can_pickup_actor.h"
#include "frc971/actors/horizontal_can_pickup_actor.h"

using ::frc971::control_loops::claw_queue;
using ::frc971::control_loops::drivetrain_queue;
using ::frc971::control_loops::fridge_queue;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace frc971 {
namespace input {
namespace joysticks {

// Actions needed.

// Human Player Station Intake Button
// Claw open
// Claw close
// Claw down

// Stack + Lift (together)
// Place

// Hold stack

// Horizontal can pickup
// Vertical can pickup

// TODO(austin): Pull a lot of the constants below out up here so they can be
// globally changed easier.

// preset motion limits
constexpr actors::ProfileParams kArmMove{1.00, 1.0};
constexpr actors::ProfileParams kElevatorMove{1.00, 3.2};

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

//const ButtonLocation kClawClosed(3, 5);
//const ButtonLocation kFridgeClosed(3, 1);


const ButtonLocation kRollersIn(4, 5);
const ButtonLocation kClawToggle(4, 1);

const POVLocation kElevatorCanUp(4, 0);

// POV stick 3, 180
const POVLocation kCanPickup(4, 180);
const ButtonLocation kToteChute(4, 6);
const ButtonLocation kStackAndLift(4, 7);
const ButtonLocation kStackAndHold(3, 5);

// Pull in the 6th tote.
//const ButtonLocation kSixthTote(4, 10);
const ButtonLocation kCanUp(4, 10);

const ButtonLocation kHeldToLift(4, 11);
const ButtonLocation kPickup(4, 9);

const ButtonLocation kStack(4, 2);

// Move the fridge out with the stack in preparation for scoring.
const ButtonLocation kScore(4, 8);
const ButtonLocation kCoopTop(3, 8);
const ButtonLocation kCoopTopRetract(3, 7);
const ButtonLocation kCoopBottom(3, 6);
const ButtonLocation kCoopBottomRetract(3, 9);

const ButtonLocation kRetractFromScore(4, 12);

const POVLocation kFridgeToggle(4, 270);
const ButtonLocation kSpit(4, 3);

// Set stack down in the bot.
const POVLocation kSetStackDownAndHold(4, 90);

const double kClawTotePackAngle = 0.95;
const double kArmRaiseLowerClearance = -0.08;

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : was_running_(false) {}

  static actors::ScoreParams MakeScoreParams() {
    actors::ScoreParams r;
    r.move_the_stack = r.place_the_stack = true;
    r.upper_move_height = 0.14;
    r.begin_horizontal_move_height = 0.13;
    r.horizontal_move_target = -0.7;
    r.place_height = -0.10;
    r.home_return_height = 0.1;
    return r;
  }

  static actors::ScoreParams MakeCoopTopParams(bool place_the_stack) {
    actors::ScoreParams r;
    r.move_the_stack = !place_the_stack;
    r.place_the_stack = place_the_stack;
    r.upper_move_height = 0.52;
    r.begin_horizontal_move_height = 0.5;
    r.horizontal_move_target = -0.48;
    r.place_height = 0.39;
    r.home_return_height = 0.1;
    return r;
  }

  static actors::ScoreParams MakeCoopBottomParams(bool place_the_stack) {
    actors::ScoreParams r;
    r.move_the_stack = !place_the_stack;
    r.place_the_stack = place_the_stack;
    r.upper_move_height = 0.17;
    r.begin_horizontal_move_height = 0.16;
    r.horizontal_move_target = -0.7;
    r.place_height = 0.0;
    r.home_return_height = 0.1;
    return r;
  }

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    bool last_auto_running = auto_running_;
    auto_running_ = data.GetControlBit(ControlBit::kAutonomous) &&
                    data.GetControlBit(ControlBit::kEnabled);
    if (auto_running_ != last_auto_running) {
      if (auto_running_) {
        StartAuto();
      } else {
        StopAuto();
      }
    }

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);

    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(false)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    double intake_power = 0.0;
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    if (data.IsPressed(kRollersIn)) {
      intake_power = 10.0;
      claw_goal_ = 0.0;
    }
    if (data.IsPressed(kSpit)) {
      intake_power = -12.0;
    }

    // Toggle button for the claw
    if (data.PosEdge(kClawToggle)) {
      claw_rollers_closed_ = !claw_rollers_closed_;
    }

    /*
    if (data.IsPressed(kClawClosed)) {
      claw_rollers_closed_ = true;
    }
    */

    // Horizontal can pickup.
    if (data.PosEdge(kElevatorCanUp)) {
      actors::HorizontalCanPickupParams params;
      params.elevator_height = 0.3;
      params.pickup_angle = 0.40;
      params.suck_time = 0.08;
      params.suck_power = 8.0;

      params.claw_settle_time = 0.05;
      params.claw_settle_power = 5.0;
      params.claw_full_lift_angle = 1.35;
      params.claw_end_angle = 0.5;

      // End low so we don't drop it.
      params.elevator_end_height = 0.3;
      params.arm_end_angle = 0.0;
      action_queue_.EnqueueAction(
          actors::MakeHorizontalCanPickupAction(params));
      fridge_closed_ = true;
    }

    // -0.942503 arm, 0.374752 elevator
    // Vertical can pickup.
    if (data.PosEdge(kCanPickup)) {
      actors::CanPickupParams params;
      params.pickup_angle = -0.93;
      params.pickup_height = 0.265;
      params.lift_height = 0.65;
      // End low so the can is supported.
      params.end_height = 0.3;
      params.end_angle = 0.0;
      action_queue_.EnqueueAction(actors::MakeCanPickupAction(params));
      fridge_closed_ = true;
    }

    // Tote chute pull in when button is pressed, pack when done.
    if (data.IsPressed(kToteChute)) {
      claw_goal_ = 0.8;
      intake_power = 7.0;
    }
    if (data.NegEdge(kToteChute)) {
      claw_goal_ = kClawTotePackAngle;
    }

    if (data.PosEdge(kStackAndLift)) {
      actors::StackAndLiftParams params;
      params.stack_params.claw_out_angle = 0.6;
      params.stack_params.bottom = 0.020;
      params.stack_params.over_box_before_place_height = 0.39;
      params.stack_params.arm_clearance = kArmRaiseLowerClearance;

      params.grab_after_stack = true;
      params.clamp_pause_time = 0.1;
      params.lift_params.lift_height = 0.45;
      params.lift_params.lift_arm = 0.3;
      params.grab_after_lift = true;
      fridge_closed_ = true;

      action_queue_.EnqueueAction(actors::MakeStackAndLiftAction(params));
    }

    if (data.PosEdge(kStackAndHold) || data.PosEdge(kSetStackDownAndHold)) {
      actors::StackAndHoldParams params;
      params.bottom = 0.020;
      params.over_box_before_place_height = 0.39;

      params.arm_clearance = kArmRaiseLowerClearance;
      params.clamp_pause_time = 0.25;
      params.claw_clamp_angle = kClawTotePackAngle;

      params.hold_height = 0.68;
      params.claw_out_angle = 0.6;

      if (data.PosEdge(kSetStackDownAndHold)) {
        params.place_not_stack = true;
        params.clamp_pause_time = 0.1;
        //params.claw_clamp_angle = kClawTotePackAngle - 0.5;
      } else {
        params.place_not_stack = false;
      }

      action_queue_.EnqueueAction(actors::MakeStackAndHoldAction(params));
      fridge_closed_ = true;
    }

    // TODO(austin): Figure out what action needed to pull the 6th tote into the
    // claw.

    // Lower the fridge from holding the stack, grab the stack, and then lift.
    if (data.PosEdge(kHeldToLift)) {
      actors::HeldToLiftParams params;
      params.arm_clearance = kArmRaiseLowerClearance;
      params.clamp_pause_time = 0.1;
      params.before_lift_settle_time = 0.1;
      params.bottom_height = 0.020;
      params.claw_out_angle = 0.6;
      params.lift_params.lift_height = 0.45;
      params.lift_params.lift_arm = 0.3;
      fridge_closed_ = true;

      action_queue_.EnqueueAction(actors::MakeHeldToLiftAction(params));
    }

    // Lift the can up.
    if (data.PosEdge(kCanUp)) {
      actors::LiftParams params;
      params.lift_height = 0.68;
      params.lift_arm = 0.3;
      fridge_closed_ = true;

      action_queue_.EnqueueAction(actors::MakeLiftAction(params));
    }

    // Pick up a tote from the ground and put it in the bottom of the bot.
    if (data.PosEdge(kPickup)) {
      actors::PickupParams params;
      // Lift to here initially.
      params.pickup_angle = 0.9;
      // Start sucking here
      params.suck_angle = 0.8;
      // Go back down to here to finish sucking.
      params.suck_angle_finish = 0.4;
      // Pack the box back in here.
      params.pickup_finish_angle = kClawTotePackAngle;
      params.intake_time = 0.8;
      params.intake_voltage = 7.0;
      action_queue_.EnqueueAction(actors::MakePickupAction(params));
    }

    // Place stack on a tote in the tray, and grab it.
    if (data.PosEdge(kStack)) {
      actors::StackParams params;
      params.claw_out_angle = 0.6;
      params.bottom = 0.020;
      params.only_place = false;
      params.arm_clearance = kArmRaiseLowerClearance;
      params.over_box_before_place_height = 0.39;
      action_queue_.EnqueueAction(actors::MakeStackAction(params));
      claw_rollers_closed_ = true;
      fridge_closed_ = true;
    }

    if (data.PosEdge(kScore)) {
      action_queue_.EnqueueAction(
          actors::MakeScoreAction(MakeScoreParams()));
      fridge_closed_ = false;
    }

    if (data.PosEdge(kCoopTop)) {
      action_queue_.EnqueueAction(
          actors::MakeScoreAction(MakeCoopTopParams(false)));
    }
    if (data.PosEdge(kCoopTopRetract)) {
      action_queue_.EnqueueAction(
          actors::MakeScoreAction(MakeCoopTopParams(true)));
      fridge_closed_ = false;
    }

    if (data.PosEdge(kCoopBottom)) {
      action_queue_.EnqueueAction(
          actors::MakeScoreAction(MakeCoopBottomParams(false)));
    }
    if (data.PosEdge(kCoopBottomRetract)) {
      action_queue_.EnqueueAction(
          actors::MakeScoreAction(MakeCoopBottomParams(true)));
      fridge_closed_ = false;
    }

    if (data.PosEdge(kFridgeToggle)) {
      fridge_closed_ = !fridge_closed_;
    }

    if (data.PosEdge(ControlBit::kEnabled)) {
      // If we got enabled, wait for everything to zero.
      LOG(INFO, "Waiting for zero.\n");
      waiting_for_zero_ = true;
    }

    claw_queue.status.FetchLatest();
    fridge_queue.status.FetchLatest();
    if (!claw_queue.status.get()) {
      LOG(ERROR, "Got no claw status packet.\n");
      // Not safe to continue.
      return;
    }
    if (!fridge_queue.status.get()) {
      LOG(ERROR, "Got no fridge status packet.\n");
      return;
    }

    if (claw_queue.status->zeroed && fridge_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(INFO, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;

        // Set the initial goals to where we are now.
        elevator_goal_ = fridge_queue.status->goal_height;
        arm_goal_ = fridge_queue.status->goal_angle;
        claw_goal_ = claw_queue.status->angle;
      }
    } else {
      waiting_for_zero_ = true;
      return;
    }

    if (!waiting_for_zero_) {
      if (!action_queue_.Running()) {
        auto new_fridge_goal = fridge_queue.goal.MakeMessage();
        new_fridge_goal->max_velocity = elevator_params_.velocity;
        new_fridge_goal->max_acceleration = elevator_params_.acceleration;
        new_fridge_goal->profiling_type = 0;
        new_fridge_goal->height = elevator_goal_;
        new_fridge_goal->velocity = 0.0;
        new_fridge_goal->max_angular_velocity = arm_params_.velocity;
        new_fridge_goal->max_angular_acceleration = arm_params_.acceleration;
        new_fridge_goal->angle = arm_goal_;
        new_fridge_goal->angular_velocity = 0.0;
        new_fridge_goal->grabbers.top_front = fridge_closed_;
        new_fridge_goal->grabbers.top_back = fridge_closed_;
        new_fridge_goal->grabbers.bottom_front = fridge_closed_;
        new_fridge_goal->grabbers.bottom_back = fridge_closed_;

        if (!new_fridge_goal.Send()) {
          LOG(ERROR, "Sending fridge goal failed.\n");
        } else {
          LOG(DEBUG, "sending goals: elevator: %f, arm: %f\n", elevator_goal_,
              arm_goal_);
        }
        if (!claw_queue.goal.MakeWithBuilder()
                 .angle(claw_goal_)
                 .rollers_closed(claw_rollers_closed_)
                 .max_velocity(4.0)
                 .max_acceleration(6.0)
                 .intake(intake_power)
                 .Send()) {
          LOG(ERROR, "Sending claw goal failed.\n");
        }
      }
    }

    if (action_queue_.Running()) {
      // If we are running an action, update our goals to the current goals.
      control_loops::fridge_queue.status.FetchLatest();
      if (control_loops::fridge_queue.status.get()) {
        arm_goal_ = control_loops::fridge_queue.status->goal_angle;
        elevator_goal_ = control_loops::fridge_queue.status->goal_height;
      } else {
        LOG(ERROR, "No fridge status!\n");
      }

      // If we are running an action, update our goals to the current goals.
      control_loops::claw_queue.status.FetchLatest();
      if (control_loops::claw_queue.status.get()) {
        claw_goal_ = control_loops::claw_queue.status->goal_angle;
      } else {
        LOG(ERROR, "No fridge status!\n");
      }
    }
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");
    ::frc971::autonomous::autonomous.MakeWithBuilder().run_auto(true).Send();
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    ::frc971::autonomous::autonomous.MakeWithBuilder().run_auto(false).Send();
  }

  bool was_running_;

  // Previous goals for systems.
  double elevator_goal_ = 0.2;
  double arm_goal_ = 0.0;
  double claw_goal_ = 0.0;
  bool claw_rollers_closed_ = false;
  bool fridge_closed_ = false;
  actors::ProfileParams arm_params_ = kArmMove;
  actors::ProfileParams elevator_params_ = kElevatorMove;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  bool auto_running_ = false;

  ::aos::common::actions::ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
