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
#include "y2016/vision/vision.q.h"

#include "y2016/constants.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/autonomous/auto.q.h"
#include "y2016/actors/autonomous_actor.h"
#include "y2016/actors/vision_align_actor.h"
#include "y2016/actors/superstructure_actor.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"

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
const ButtonLocation kIntakeIn(3, 12);
const ButtonLocation kFire(3, 3);
const ButtonLocation kIntakeOut(3, 9);
const ButtonLocation kPortcullis(3, 7);
const ButtonLocation kChevalDeFrise(3, 8);

const ButtonLocation kVisionAlign(3, 4);

const ButtonLocation kExpand(3, 6);
const ButtonLocation kWinch(3, 5);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(true),
        intake_goal_(0.0),
        shoulder_goal_(M_PI / 2.0),
        wrist_goal_(0.0),
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
      is_high_gear_ = true;
    }

    vision_valid_ = false;

    ::y2016::vision::vision_status.FetchLatest();

    if (::y2016::vision::vision_status.get()) {
      vision_valid_ = (::y2016::vision::vision_status->left_image_valid &&
                      ::y2016::vision::vision_status->right_image_valid);
      last_angle_ = ::y2016::vision::vision_status->horizontal_angle;
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

    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);
    drivetrain_queue.status.FetchLatest();

    if (data.IsPressed(kVisionAlign)) {
      if (vision_valid_ && !vision_action_running_) {
        actors::VisionAlignActionParams params;
        action_queue_.EnqueueAction(actors::MakeVisionAlignAction(params));
        vision_action_running_ = true;
        LOG(INFO, "Starting vision align\n");
      } else {
        if (!vision_valid_) {
          LOG(INFO, "Vision align but not valid\n");
        }
      }
    }

    if (data.NegEdge(kVisionAlign)) {
      action_queue_.CancelAllActions();
    }
    if (!data.IsPressed(kVisionAlign)) {
      vision_action_running_ = false;
    }

    // Don't do any normal drivetrain stuff if vision is in charge.
    if (vision_action_running_) {
      return;
    }

    if (data.PosEdge(kTurn1) || data.PosEdge(kTurn2)) {
      if (drivetrain_queue.status.get()) {
        left_goal_ = drivetrain_queue.status->estimated_left_position;
        right_goal_ = drivetrain_queue.status->estimated_right_position;
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
             .left_goal(left_goal_ - wheel * 0.5 + throttle * 0.3)
             .right_goal(right_goal_ + wheel * 0.5 + throttle * 0.3)
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
    float voltage_climber = 0.0;
    // Default the intake to up.
    intake_goal_ = constants::Values::kIntakeRange.upper - 0.04;

    bool force_lights_on = false;
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
    }

    if (superstructure_queue.status.get() &&
        superstructure_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(DEBUG, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;
      }
    } else {
      waiting_for_zero_ = true;
    }

    double intake_when_shooting = kMaxIntakeAngleBeforeArmInterference;
    bool use_slow_profile = false;
    if (vision_action_running_) {
      use_slow_profile = true;
      if (vision_valid_) {
        intake_when_shooting -= 0.5;
      }
    }

    if (data.IsPressed(kFrontLong)) {
      // Forwards shot
      shoulder_goal_ = M_PI / 2.0 + 0.1;
      wrist_goal_ = M_PI + 0.41 + 0.02 - 0.005;
      if (drivetrain_queue.status.get()) {
        wrist_goal_ += drivetrain_queue.status->ground_angle;
      }
      shooter_velocity_ = 640.0;
      intake_goal_ = intake_when_shooting;
    } else if (data.IsPressed(kBackLong)) {
      // Backwards shot
      shoulder_goal_ = M_PI / 2.0 - 0.4;
      wrist_goal_ = -0.62 - 0.02;
      if (drivetrain_queue.status.get()) {
        wrist_goal_ += drivetrain_queue.status->ground_angle;
      }
      shooter_velocity_ = 640.0;
      intake_goal_ = intake_when_shooting;
    } else if (data.IsPressed(kBackFender)) {
      // Backwards shot no IMU
      shoulder_goal_ = M_PI / 2.0 - 0.4;
      wrist_goal_ = -0.62 - 0.02;
      shooter_velocity_ = 640.0;
      intake_goal_ = intake_when_shooting;
    } else if (data.IsPressed(kFrontFender)) {
      // Forwards shot no IMU
      shoulder_goal_ = M_PI / 2.0 + 0.1;
      wrist_goal_ = M_PI + 0.41 + 0.02 - 0.005;
      shooter_velocity_ = 640.0;
      intake_goal_ = intake_when_shooting;
    } else if (data.IsPressed(kExpand) || data.IsPressed(kWinch)) {
      // Set the goals to the hanging position so when the actor finishes, we
      // will still be at the right spot.
      shoulder_goal_ = 1.2;
      wrist_goal_ = 1.0;
      intake_goal_ = 0.0;
      if (data.PosEdge(kExpand)) {
        is_expanding_ = true;
        actors::SuperstructureActionParams params;
        params.partial_angle = 0.3;
        params.delay_time = 0.7;
        params.full_angle = shoulder_goal_;
        params.shooter_angle = wrist_goal_;
        action_queue_.EnqueueAction(actors::MakeSuperstructureAction(params));
      }
      if (data.IsPressed(kWinch)) {
        voltage_climber = 12.0;
      }
    } else {
      wrist_goal_ = 0.0;
      shoulder_goal_ = -0.010;
      shooter_velocity_ = 0.0;
    }
    if (data.NegEdge(kExpand) || voltage_climber > 1.0) {
      is_expanding_ = false;
      action_queue_.CancelAllActions();
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

    fire_ = false;
    if (data.IsPressed(kFire) && shooter_velocity_ != 0.0) {
      if (data.IsPressed(kVisionAlign)) {
        // Make sure that we are lined up.
        drivetrain_queue.status.FetchLatest();
        drivetrain_queue.goal.FetchLatest();
        if (drivetrain_queue.status.get() && drivetrain_queue.goal.get()) {
          const double left_goal = drivetrain_queue.goal->left_goal;
          const double right_goal = drivetrain_queue.goal->right_goal;
          const double left_current =
              drivetrain_queue.status->estimated_left_position;
          const double right_current =
              drivetrain_queue.status->estimated_right_position;
          const double left_velocity =
              drivetrain_queue.status->estimated_left_velocity;
          const double right_velocity =
              drivetrain_queue.status->estimated_right_velocity;
          if (vision_action_running_ && ::std::abs(last_angle_) < 0.02 &&
              ::std::abs((left_goal - right_goal) -
                         (left_current - right_current)) /
                      dt_config_.robot_radius / 2.0 <
                  0.02 &&
              ::std::abs(left_velocity - right_velocity) < 0.01) {
            ++ready_to_fire_;
          } else {
            ready_to_fire_ = 0;
          }
          if (ready_to_fire_ > 9) {
            fire_ = true;
          }
        }
      } else {
        fire_ = true;
      }
    }

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

    if (data.IsPressed(kPortcullis)) {
      traverse_unlatched_ = true;
      traverse_down_ = true;
    } else if (data.IsPressed(kChevalDeFrise)) {
      traverse_unlatched_ = false;
      traverse_down_ = true;
    } else {
      traverse_unlatched_ = true;
      traverse_down_ = false;
    }

    if (!waiting_for_zero_) {
      if (!is_expanding_) {
        auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();
        new_superstructure_goal->angle_intake = intake_goal_;
        new_superstructure_goal->angle_shoulder = shoulder_goal_;
        new_superstructure_goal->angle_wrist = wrist_goal_;

        new_superstructure_goal->max_angular_velocity_intake = 7.0;
        new_superstructure_goal->max_angular_velocity_shoulder = 4.0;
        new_superstructure_goal->max_angular_velocity_wrist = 10.0;
        if (use_slow_profile) {
          new_superstructure_goal->max_angular_acceleration_intake = 10.0;
        } else {
          new_superstructure_goal->max_angular_acceleration_intake = 40.0;
        }
        new_superstructure_goal->max_angular_acceleration_shoulder = 10.0;
        if (shoulder_goal_ > 1.0) {
          new_superstructure_goal->max_angular_acceleration_wrist = 45.0;
        } else {
          new_superstructure_goal->max_angular_acceleration_wrist = 25.0;
        }

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

        new_superstructure_goal->traverse_unlatched = traverse_unlatched_;
        new_superstructure_goal->unfold_climber = false;
        new_superstructure_goal->voltage_climber = voltage_climber;
        new_superstructure_goal->traverse_down = traverse_down_;
        new_superstructure_goal->force_intake = true;

        if (!new_superstructure_goal.Send()) {
          LOG(ERROR, "Sending superstructure goal failed.\n");
        } else {
          LOG(DEBUG, "sending goals: intake: %f, shoulder: %f, wrist: %f\n",
              intake_goal_, shoulder_goal_, wrist_goal_);
        }
      }

      if (!shooter_queue.goal.MakeWithBuilder()
               .angular_velocity(shooter_velocity_)
               .clamp_open(is_intaking_ || is_outtaking_)
               .push_to_shooter(fire_)
               .force_lights_on(force_lights_on)
               .shooting_forwards(wrist_goal_ > 0)
               .Send()) {
        LOG(ERROR, "Sending shooter goal failed.\n");
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

  bool is_high_gear_;
  // Whatever these are set to are our default goals to send out after zeroing.
  double intake_goal_;
  double shoulder_goal_;
  double wrist_goal_;
  double shooter_velocity_ = 0.0;

  // Turning goals
  double left_goal_;
  double right_goal_;

  bool was_running_ = false;
  bool auto_running_ = false;

  bool traverse_unlatched_ = false;
  bool traverse_down_ = false;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  // If true, the ball was present when the intaking button was pressed.
  bool saw_ball_when_started_intaking_ = false;

  bool is_intaking_ = false;
  bool is_outtaking_ = false;
  bool fire_ = false;

  bool vision_action_running_ = false;
  bool vision_valid_ = false;

  int recently_intaking_accumulator_ = 0;
  double last_angle_ = 100;

  int ready_to_fire_ = 0;

  ::aos::common::actions::ActionQueue action_queue_;

  const ::frc971::control_loops::drivetrain::DrivetrainConfig dt_config_;

  bool is_expanding_ = false;

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
