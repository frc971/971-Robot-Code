#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/actions/actions.h"

#include "frc971/queues/gyro.q.h"
#include "y2015_bot3/autonomous/auto.q.h"
#include "y2015_bot3/control_loops/elevator/elevator.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2015_bot3/control_loops/elevator/elevator.q.h"
#include "y2015_bot3/control_loops/intake/intake.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2015_bot3::control_loops::elevator_queue;
using ::y2015_bot3::control_loops::intake_queue;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace y2015_bot3 {
namespace input {
namespace joysticks {

struct ProfileParams {
  double velocity;
  double acceleration;
};

// Preset motion limits.
constexpr ProfileParams kElevatorMove{0.3, 1.0};
constexpr ProfileParams kFastElevatorMove{1.0, 5.0};

// Preset goals for autostacking
constexpr double kOneToteHeight{0.46};
constexpr double kCarryHeight{0.180};
constexpr double kGroundHeight{0.030};

// Joystick & button addresses.
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 3), kShiftLow(2, 1);
const ButtonLocation kQuickTurn(1, 5);

// essential
const ButtonLocation kMoveToteHeight(4, 9);
const ButtonLocation kOpenIntake(3, 8);
const ButtonLocation kCloseIntake(3, 6);
const ButtonLocation kOpenCanRestraint(4, 5);
const ButtonLocation kCloseCanRestraint(4, 1);
const POVLocation kOpenPassiveSupport(4, 0);
const POVLocation kClosePassiveSupport(4, 180);

const ButtonLocation kIntakeOut(3, 7);
const ButtonLocation kIntakeIn(4, 12);

const ButtonLocation kAutoStack(4, 7);
const ButtonLocation kManualStack(4, 6);

const ButtonLocation kCarry(4, 10);
const ButtonLocation kSetDown(3, 9);
const ButtonLocation kSkyscraper(4, 11);

const ButtonLocation kScoreBegin(4, 8);

const ButtonLocation kCanGrabberLift(3, 2);
const ButtonLocation kFastCanGrabberLift(2, 3);
const ButtonLocation kCanGrabberLower(3, 5);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : was_running_(false) {}

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
      intake_closed_ = true;
      can_restraint_open_ = false;
      passive_support_open_ = true;
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
    double intake_goal = 0;

    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    elevator_queue.status.FetchLatest();
    if (!elevator_queue.status.get()) {
      LOG(ERROR, "Got no elevator status packet.\n");
    }

    if (elevator_queue.status.get() && elevator_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(INFO, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;

        // Set the initial goals to the bottom, since otherwise the driver seems
        // to get confused and think that the end of the zeroing sequence is the
        // same location and then wonders why it doesn't work...
        elevator_goal_ = kGroundHeight;
      }
    } else {
      waiting_for_zero_ = true;
    }

    if (data.PosEdge(kAutoStack)) {
      action_queue_.CancelAllActions();
      if (tote_count_ == 6) {
        stacking_state_machine_ = FULL;
      } else {
        stacking_state_machine_ = WAITING;
      }
    }
    if (data.IsPressed(kAutoStack)) {
      switch (stacking_state_machine_) {
        case WAITING:
          elevator_params_ = kFastElevatorMove;
          elevator_goal_ = kOneToteHeight;
          if (elevator_queue.status->has_tote) {
            stacking_state_machine_ = GOING_DOWN;
          }
          break;
        case GOING_DOWN:
          elevator_params_ = kFastElevatorMove;
          elevator_goal_ = kGroundHeight;
          if (elevator_queue.status->height < 0.05) {
            ++tote_count_;
            if (tote_count_ == 6) {
              stacking_state_machine_ = FULL;
            } else {
              stacking_state_machine_ = GOING_UP;
            }
          }
          break;
        case GOING_UP:
          elevator_params_ = kFastElevatorMove;
          elevator_goal_ = kOneToteHeight;
          if (elevator_queue.status->height > kOneToteHeight - 0.05) {
            stacking_state_machine_ = WAITING;
          }
          break;
        case FULL:
          elevator_goal_ = kCarryHeight;
          elevator_params_ = kFastElevatorMove;
          break;
        case OFF:
          stacking_state_machine_ = WAITING;
          break;
      }
    } else {
      stacking_state_machine_ = OFF;
    }

    // Buttons for intaking.
    if (data.IsPressed(kIntakeIn)) {
      intake_goal = 10.0;
      if (stacking_state_machine_ != OFF &&
          elevator_queue.status->height < 0.43) {
        intake_goal = 0.0;
      }
    } else if (data.IsPressed(kIntakeOut)) {
      intake_goal = -10.0;

      action_queue_.CancelAllActions();
    }
    // TODO(Adam): Implement essential actors/goals.
    if (data.PosEdge(kMoveToteHeight)) {
      elevator_goal_ = 0.45;
      elevator_params_ = {1.0, 5.0};
      action_queue_.CancelAllActions();
    }

    if (data.IsPressed(kOpenIntake)) {
      intake_closed_ = false;
    }

    if (data.IsPressed(kCloseIntake)) {
      intake_closed_ = true;
    }

    if (data.IsPressed(kOpenCanRestraint)) {
      can_restraint_open_ = true;
    }

    if (data.IsPressed(kCloseCanRestraint)) {
      can_restraint_open_ = false;
    }

    if (data.IsPressed(kOpenPassiveSupport)) {
      passive_support_open_ = true;
    }

    if (data.IsPressed(kClosePassiveSupport)) {
      passive_support_open_ = false;
    }

    // Buttons for elevator.
    if (data.IsPressed(kCarry)) {
      // TODO(comran): Get actual height/velocity/acceleration values.
      elevator_goal_ = 0.180;
      elevator_params_ = {1.0, 2.0};
      action_queue_.CancelAllActions();
    }

    if (data.IsPressed(kSetDown)) {
      // TODO(comran): Get actual height/velocity/acceleration values.
      elevator_goal_ = 0.005;
      elevator_params_ = {1.0, 5.0};
      action_queue_.CancelAllActions();
    }

    if (data.IsPressed(kSkyscraper)) {
      // TODO(comran): Get actual height/velocity/acceleration values.
      elevator_goal_ = 1.0;
      elevator_params_ = {1.0, 5.0};
    }

    if (data.IsPressed(kScoreBegin)) {
      // TODO(comran): Get actual height/velocity/acceleration values.
      elevator_goal_ = 0.005;
      elevator_params_ = {1.0, 5.0};
      intake_closed_ = false;
      can_restraint_open_ = true;
      passive_support_open_ = true;
      tote_count_ = 0;
    }

    // Buttons for can grabber.
    if (data.IsPressed(kCanGrabberLift)) {
      ::y2015_bot3::autonomous::can_grabber_control.MakeWithBuilder()
          .can_grabber_voltage(-3)
          .can_grabbers(false)
          .Send();
    } else if (data.IsPressed(kFastCanGrabberLift)) {
      ::y2015_bot3::autonomous::can_grabber_control.MakeWithBuilder()
          .can_grabber_voltage(-12).can_grabbers(false).Send();
    } else if (data.IsPressed(kCanGrabberLower)) {
      if (grab_delay_ > 5) {
        ::y2015_bot3::autonomous::can_grabber_control.MakeWithBuilder()
            .can_grabber_voltage(2).can_grabbers(true).Send();
      } else {
        ::y2015_bot3::autonomous::can_grabber_control.MakeWithBuilder()
            .can_grabber_voltage(0).can_grabbers(true).Send();
      }
      ++grab_delay_;
    } else {
      grab_delay_ = 0;
    }

    // Send our goals if everything looks OK.
    if (!waiting_for_zero_) {
      if (!action_queue_.Running()) {
        // Send our elevator goals, with limits set in the profile params.
        auto new_elevator_goal = elevator_queue.goal.MakeMessage();
        new_elevator_goal->max_velocity = elevator_params_.velocity;
        new_elevator_goal->max_acceleration = elevator_params_.acceleration;
        new_elevator_goal->height = elevator_goal_;
        new_elevator_goal->velocity = 0.0;
        new_elevator_goal->passive_support = passive_support_open_;
        new_elevator_goal->can_support = can_restraint_open_;

        if (new_elevator_goal.Send()) {
          LOG(DEBUG, "sending goals: elevator: %f\n", elevator_goal_);
        } else {
          LOG(ERROR, "Sending elevator goal failed.\n");
        }
      }
    }

    // Send our intake goals.
    if (!intake_queue.goal.MakeWithBuilder().movement(intake_goal)
            .claw_closed(intake_closed_).Send()) {
      LOG(ERROR, "Sending intake goal failed.\n");
    }

    // If an action is running, use the action's goals for the profiled
    // superstructure subsystems & bypass others.
    if (action_queue_.Running()) {
      control_loops::elevator_queue.status.FetchLatest();
      if (control_loops::elevator_queue.status.get()) {
        elevator_goal_ = control_loops::elevator_queue.status->goal_height;
      } else {
        LOG(ERROR, "No elevator status!\n");
      }
    }
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");
    ::y2015_bot3::autonomous::autonomous.MakeWithBuilder().run_auto(true).Send();
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    ::y2015_bot3::autonomous::autonomous.MakeWithBuilder().run_auto(false).Send();
  }

  bool was_running_;

  double elevator_goal_ = 0.2;

  ProfileParams elevator_params_ = kElevatorMove;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  bool auto_running_ = false;

  bool intake_closed_ = false;

  bool can_restraint_open_ = true;

  bool passive_support_open_ = true;

  int tote_count_ = 0;

  ::aos::common::actions::ActionQueue action_queue_;

  enum StackingStateMachine {
    WAITING,
    GOING_DOWN,
    GOING_UP,
    FULL,
    OFF
  };

  StackingStateMachine stacking_state_machine_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::std::chrono::milliseconds(200), WARNING,
                                     "no drivetrain status");
  int grab_delay_ = 0;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2015_bot3

int main() {
  ::aos::Init(-1);
  ::y2015_bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
