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
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/control_loops/superstructure/superstructure.h"
#include "y2016/queues/ball_detector.q.h"

#include "y2016/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::y2016::control_loops::shooter::shooter_queue;
using ::y2016::control_loops::superstructure_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace y2016 {
namespace input {
namespace joysticks {

namespace {

constexpr double kMaxIntakeAngleBeforeArmInterference = control_loops::
    superstructure::CollisionAvoidance::kMaxIntakeAngleBeforeArmInterference;

}  // namespace

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 3), kShiftHigh2(2, 2), kShiftLow(2, 1);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kTurn1(1, 7);
const ButtonLocation kTurn2(1, 11);

// Buttons on the lexan driver station to get things running on bring-up day.
const ButtonLocation kIntakeDown(3, 11);
const POVLocation kFrontLong(3, 180);
const POVLocation kBackLong(3, 0);
const POVLocation kBackFender(3, 90);
const POVLocation kFrontFender(3, 270);
const ButtonLocation kTest3(3, 7);
const ButtonLocation kIntakeIn(3, 12);
const ButtonLocation kTest5(3, 8);
const ButtonLocation kFire(3, 3);
const ButtonLocation kTest7(3, 5);
const ButtonLocation kIntakeOut(3, 9);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        intake_goal_(0.0),
        shoulder_goal_(M_PI / 2.0),
        wrist_goal_(0.0) {}

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

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    bool is_control_loop_driving = false;
    static double left_goal = 0.0;
    static double right_goal = 0.0;

    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);

    if (data.PosEdge(kTurn1) || data.PosEdge(kTurn2)) {
      drivetrain_queue.status.FetchLatest();
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
             .highgear(is_high_gear_)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal - wheel * 0.5 + throttle * 0.3)
             .right_goal(right_goal + wheel * 0.5 + throttle * 0.3)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }

    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = false;
    }

    if (data.PosEdge(kShiftHigh) || data.PosEdge(kShiftHigh2)) {
      is_high_gear_ = true;
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    // Default the intake to up.
    intake_goal_ = constants::Values::kIntakeRange.upper - 0.04;

    bool force_lights_on = false;
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    if (data.PosEdge(ControlBit::kEnabled)) {
      // If we got enabled, wait for everything to zero.
      LOG(INFO, "Waiting for zero.\n");
      waiting_for_zero_ = true;
      is_high_gear_ = true;
    }

    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
    }

    if (superstructure_queue.status.get() &&
        superstructure_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(INFO, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;
      }
    } else {
      waiting_for_zero_ = true;
    }

    if (data.IsPressed(kFrontLong)) {
      // Forwards shot
      shoulder_goal_ = M_PI / 2.0 - 0.2;
      wrist_goal_ = M_PI + 0.42;
      shooter_velocity_ = 640.0;
      intake_goal_ = kMaxIntakeAngleBeforeArmInterference;
    } else if (data.IsPressed(kBackLong)) {
      // Backwards shot
      shoulder_goal_ = M_PI / 2.0 - 0.2;
      wrist_goal_ = -0.59;
      shooter_velocity_ = 640.0;
      intake_goal_ = kMaxIntakeAngleBeforeArmInterference;
    } else if (data.IsPressed(kBackFender)) {
      // Fender shot back
      shoulder_goal_ = 0.65;
      wrist_goal_ = -1.0;
      shooter_velocity_ = 550.0;
      intake_goal_ = kMaxIntakeAngleBeforeArmInterference;
    } else if (data.IsPressed(kFrontFender)) {
      // Fender shot back
      shoulder_goal_ = 1.45;
      wrist_goal_ = 2.5 + 1.7;
      shooter_velocity_ = 550.0;
      intake_goal_ = kMaxIntakeAngleBeforeArmInterference;
    } else {
      wrist_goal_ = 0.0;
      shoulder_goal_ = -0.010;
      shooter_velocity_ = 0.0;
    }

    if (data.IsPressed(kTest3)) {
      wrist_goal_ = 0.0;
    }

    bool ball_detected = false;
    ::y2016::sensors::ball_detector.FetchLatest();
    if (::y2016::sensors::ball_detector.get()) {
      ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
    }
    if (data.PosEdge(kIntakeIn)) {
      saw_ball_when_started_intaking_ = ball_detected;
    }

    if (data.IsPressed(kIntakeIn)) {
      is_intaking_ = (!ball_detected || saw_ball_when_started_intaking_);
      if (ball_detected) {
        force_lights_on = true;
      }
    } else {
      is_intaking_ = false;
    }

    if (data.IsPressed(kIntakeDown)) {
      if (is_intaking_) {
        intake_goal_ = 0.1;
      } else {
        intake_goal_ = -0.05;
      }
    }

    if (data.IsPressed(kFire) && shooter_velocity_ != 0.0) {
      fire_ = true;
    } else {
      fire_ = false;
    }

    if (data.PosEdge(kTest7)) {
    }

    is_outtaking_ = data.IsPressed(kIntakeOut);

    if (!waiting_for_zero_) {
      if (!action_queue_.Running()) {
        auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
        new_superstructure_goal->angle_intake = intake_goal_;
        new_superstructure_goal->angle_shoulder = shoulder_goal_;
        new_superstructure_goal->angle_wrist = wrist_goal_;

        new_superstructure_goal->max_angular_velocity_intake = 7.0;
        new_superstructure_goal->max_angular_velocity_shoulder = 4.0;
        new_superstructure_goal->max_angular_velocity_wrist = 10.0;
        new_superstructure_goal->max_angular_acceleration_intake = 40.0;
        new_superstructure_goal->max_angular_acceleration_shoulder = 10.0;
        new_superstructure_goal->max_angular_acceleration_wrist = 25.0;

        // Granny mode
        /*
        new_superstructure_goal->max_angular_velocity_intake = 0.2;
        new_superstructure_goal->max_angular_velocity_shoulder = 0.2;
        new_superstructure_goal->max_angular_velocity_wrist = 0.2;
        new_superstructure_goal->max_angular_acceleration_intake = 1.0;
        new_superstructure_goal->max_angular_acceleration_shoulder = 1.0;
        new_superstructure_goal->max_angular_acceleration_wrist = 1.0;
        */
        if (is_intaking_) {
          new_superstructure_goal->voltage_top_rollers = 12.0;
          new_superstructure_goal->voltage_bottom_rollers = 12.0;
        } else if (is_outtaking_) {
          new_superstructure_goal->voltage_top_rollers = -12.0;
          new_superstructure_goal->voltage_bottom_rollers = -7.0;
        } else {
          new_superstructure_goal->voltage_top_rollers = 0.0;
          new_superstructure_goal->voltage_bottom_rollers = 0.0;
        }

        if (!new_superstructure_goal.Send()) {
          LOG(ERROR, "Sending superstructure goal failed.\n");
        } else {
          LOG(DEBUG, "sending goals: intake: %f, shoulder: %f, wrist: %f\n",
              intake_goal_, shoulder_goal_, wrist_goal_);
        }

        if (!shooter_queue.goal.MakeWithBuilder()
                 .angular_velocity(shooter_velocity_)
                 .clamp_open(is_intaking_ || is_outtaking_)
                 .push_to_shooter(fire_)
                 .force_lights_on(force_lights_on)
                 .Send()) {
          LOG(ERROR, "Sending shooter goal failed.\n");
        }
      }
    }

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

  bool is_high_gear_;
  // Whatever these are set to are our default goals to send out after zeroing.
  double intake_goal_;
  double shoulder_goal_;
  double wrist_goal_;
  double shooter_velocity_ = 0.0;

  bool was_running_ = false;
  bool auto_running_ = false;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  // If true, the ball was present when the intaking button was pressed.
  bool saw_ball_when_started_intaking_ = false;

  bool is_intaking_ = false;
  bool is_outtaking_ = false;
  bool fire_ = false;

  ::aos::common::actions::ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2016

int main() {
  ::aos::Init(-1);
  ::y2016::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
