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

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"
#include "y2016_bot3/control_loops/intake/intake.h"
#include "y2016_bot3/queues/ball_detector.q.h"

#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"
#include "y2016_bot3/actors/autonomous_actor.h"
#include "y2016_bot3/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2016_bot3::control_loops::intake_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace y2016_bot3 {
namespace input {
namespace joysticks {

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kTurn1(1, 7);
const ButtonLocation kTurn2(1, 11);

// Buttons on the lexan driver station to get things running on bring-up day.
const ButtonLocation kIntakeDown(3, 5);
const ButtonLocation kIntakeIn(3, 4);
const ButtonLocation kFire(3, 3);
const ButtonLocation kIntakeOut(3, 3);
const ButtonLocation kChevalDeFrise(3, 11);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : intake_goal_(0.0),
        dt_config_(control_loops::drivetrain::GetDrivetrainConfig()) {}

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
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

    if (!data.GetControlBit(ControlBit::kEnabled)) {
      // If we are not enabled, reset the waiting for zero bit.
      LOG(DEBUG, "Waiting for zero.\n");
      waiting_for_zero_ = true;
    }

    if (!auto_running_) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }

    // Process any pending actions.
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    bool is_control_loop_driving = false;
    static double left_goal = 0.0;
    static double right_goal = 0.0;

    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);
    drivetrain_queue.status.FetchLatest();

    if (data.PosEdge(kTurn1) || data.PosEdge(kTurn2)) {
      if (drivetrain_queue.status.get()) {
        left_goal = drivetrain_queue.status->estimated_left_position;
        right_goal = drivetrain_queue.status->estimated_right_position;
      }
    }
    if (data.IsPressed(kTurn1) || data.IsPressed(kTurn2)) {
      is_control_loop_driving = true;
    }
    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal - wheel * 0.5 + throttle * 0.3)
             .right_goal(right_goal + wheel * 0.5 + throttle * 0.3)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    intake_goal_ = y2016_bot3::constants::kIntakeRange.upper - 0.04;

    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    intake_queue.status.FetchLatest();
    if (!intake_queue.status.get()) {
      LOG(ERROR, "Got no intake status packet.\n");
    }

    if (intake_queue.status.get() && intake_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(DEBUG, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;
      }
    } else {
      waiting_for_zero_ = true;
    }

    bool ball_detected = false;
    ::y2016_bot3::sensors::ball_detector.FetchLatest();
    if (::y2016_bot3::sensors::ball_detector.get()) {
      ball_detected = ::y2016_bot3::sensors::ball_detector->voltage > 2.5;
    }
    if (data.PosEdge(kIntakeIn)) {
      saw_ball_when_started_intaking_ = ball_detected;
    }

    is_intaking_ = data.IsPressed(kIntakeIn) &&
                   (!ball_detected || saw_ball_when_started_intaking_);

    is_outtaking_ = data.IsPressed(kIntakeOut);

    if (is_intaking_ || is_outtaking_) {
      recently_intaking_accumulator_ = 20;
    }

    if (data.IsPressed(kIntakeDown)) {
      if (recently_intaking_accumulator_) {
        intake_goal_ = 0.1;
      } else {
        intake_goal_ = -0.05;
      }
    }

    if (recently_intaking_accumulator_ > 0) {
      --recently_intaking_accumulator_;
    }

    if (data.IsPressed(kChevalDeFrise)) {
      traverse_down_ = true;
    } else {
      traverse_down_ = false;
    }

    if (!waiting_for_zero_) {
      auto new_intake_goal = intake_queue.goal.MakeMessage();
      new_intake_goal->angle_intake = intake_goal_;

      new_intake_goal->max_angular_velocity_intake = 7.0;
      new_intake_goal->max_angular_acceleration_intake = 40.0;

      static constexpr bool kGrannyMode = false;
      if (kGrannyMode) {
        new_intake_goal->max_angular_velocity_intake = 0.2;
        new_intake_goal->max_angular_acceleration_intake = 1.0;
      }

      if (is_intaking_) {
        new_intake_goal->voltage_intake_rollers = -12.0;
        new_intake_goal->voltage_top_rollers = -12.0;
        new_intake_goal->voltage_bottom_rollers = 12.0;
      } else if (is_outtaking_) {
        new_intake_goal->voltage_intake_rollers = 12.0;
        new_intake_goal->voltage_top_rollers = 12.0;
        new_intake_goal->voltage_bottom_rollers = -7.0;
      } else {
        new_intake_goal->voltage_intake_rollers = 0.0;
        new_intake_goal->voltage_top_rollers = 0.0;
        new_intake_goal->voltage_bottom_rollers = 0.0;
      }

      new_intake_goal->traverse_down = traverse_down_;
      new_intake_goal->force_intake = true;

      if (!new_intake_goal.Send()) {
        LOG(ERROR, "Sending intake goal failed.\n");
      } else {
        LOG(DEBUG, "sending goal: intake: %f\n", intake_goal_);
      }
    }
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");

    actors::AutonomousActionParams params;
    actors::auto_mode.FetchLatest();
    if (actors::auto_mode.get() != nullptr) {
      params.mode = actors::auto_mode->mode;
    } else {
      LOG(WARNING, "no auto mode values\n");
      params.mode = 0;
    }
    action_queue_.EnqueueAction(actors::MakeAutonomousAction(params));
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    action_queue_.CancelAllActions();
  }

  // Whatever these are set to are our default goals to send out after zeroing.
  double intake_goal_;

  bool was_running_ = false;
  bool auto_running_ = false;

  bool traverse_down_ = false;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  // If true, the ball was present when the intaking button was pressed.
  bool saw_ball_when_started_intaking_ = false;

  bool is_intaking_ = false;
  bool is_outtaking_ = false;

  int recently_intaking_accumulator_ = 0;

  ::aos::common::actions::ActionQueue action_queue_;

  const ::frc971::control_loops::drivetrain::DrivetrainConfig dt_config_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2016_bot3

int main() {
  ::aos::Init(-1);
  ::y2016_bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
