#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/log_interval.h"

#include "frc971/autonomous/auto_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2016/actors/autonomous_actor.h"
#include "y2016/actors/superstructure_actor.h"
#include "y2016/actors/vision_align_actor.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/control_loops/shooter/shooter_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"
#include "y2016/queues/ball_detector_generated.h"
#include "y2016/vision/vision_generated.h"

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

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop, control_loops::drivetrain::GetDrivetrainConfig(),
            ::aos::input::DrivetrainInputReader::InputType::kSteeringWheel, {}),
        vision_status_fetcher_(
            event_loop->MakeFetcher<::y2016::vision::VisionStatus>(
                "/superstructure")),
        ball_detector_fetcher_(
            event_loop->MakeFetcher<::y2016::sensors::BallDetector>(
                "/superstructure")),
        shooter_goal_sender_(
            event_loop->MakeSender<::y2016::control_loops::shooter::Goal>(
                "/shooter")),
        superstructure_status_fetcher_(
            event_loop
                ->MakeFetcher<::y2016::control_loops::superstructure::Status>(
                    "/superstructure")),
        superstructure_goal_sender_(
            event_loop
                ->MakeSender<::y2016::control_loops::superstructure::Goal>(
                    "/superstructure")),
        drivetrain_goal_fetcher_(
            event_loop->MakeFetcher<::frc971::control_loops::drivetrain::Goal>(
                "/drivetrain")),
        drivetrain_status_fetcher_(
            event_loop
                ->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
                    "/drivetrain")),
        intake_goal_(0.0),
        shoulder_goal_(M_PI / 2.0),
        wrist_goal_(0.0),
        vision_align_action_factory_(
            actors::VisionAlignActor::MakeFactory(event_loop)),
        superstructure_action_factory_(
            actors::SuperstructureActor::MakeFactory(event_loop)) {
    set_vision_align_fn([this](const ::aos::input::driver_station::Data &data) {
      return VisionAlign(data);
    });
  }

  // Returns true when we are vision aligning
  bool VisionAlign(const ::aos::input::driver_station::Data &data) {
    vision_valid_ = false;

    vision_status_fetcher_.Fetch();

    if (vision_status_fetcher_.get()) {
      vision_valid_ = (vision_status_fetcher_->left_image_valid() &&
                       vision_status_fetcher_->right_image_valid());
      last_angle_ = vision_status_fetcher_->horizontal_angle();
    }

    if (data.IsPressed(kVisionAlign)) {
      if (vision_valid_ && !vision_action_running_) {
        actors::vision_align_action::VisionAlignActionParamsT params;
        EnqueueAction(vision_align_action_factory_.Make(params));
        vision_action_running_ = true;
        AOS_LOG(INFO, "Starting vision align\n");
      } else {
        if (!vision_valid_) {
          AOS_LOG(INFO, "Vision align but not valid\n");
        }
      }
    }

    if (data.NegEdge(kVisionAlign)) {
      CancelAllActions();
    }
    if (!data.IsPressed(kVisionAlign)) {
      vision_action_running_ = false;
    }

    return vision_action_running_;
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      // If we are not enabled, reset the waiting for zero bit.
      AOS_LOG(DEBUG, "Waiting for zero.\n");
      waiting_for_zero_ = true;
    }

    float voltage_climber = 0.0;
    // Default the intake to up.
    intake_goal_ = constants::Values::kIntakeRange.upper - 0.04;

    bool force_lights_on = false;
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      CancelAllActions();
      AOS_LOG(DEBUG, "Canceling\n");
    }

    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status packet.\n");
    }

    if (superstructure_status_fetcher_.get() &&
        superstructure_status_fetcher_->zeroed()) {
      if (waiting_for_zero_) {
        AOS_LOG(DEBUG, "Zeroed! Starting teleop mode.\n");
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
      drivetrain_status_fetcher_.Fetch();
      if (drivetrain_status_fetcher_.get()) {
        wrist_goal_ += drivetrain_status_fetcher_->ground_angle();
      }
      shooter_velocity_ = 640.0;
      intake_goal_ = intake_when_shooting;
    } else if (data.IsPressed(kBackLong)) {
      // Backwards shot
      shoulder_goal_ = M_PI / 2.0 - 0.4;
      wrist_goal_ = -0.62 - 0.02;
      drivetrain_status_fetcher_.Fetch();
      if (drivetrain_status_fetcher_.get()) {
        wrist_goal_ += drivetrain_status_fetcher_->ground_angle();
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
        actors::superstructure_action::SuperstructureActionParamsT params;
        params.partial_angle = 0.3;
        params.delay_time = 0.7;
        params.full_angle = shoulder_goal_;
        params.shooter_angle = wrist_goal_;
        EnqueueAction(superstructure_action_factory_.Make(params));
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
      CancelAllActions();
    }

    bool ball_detected = false;
    ball_detector_fetcher_.Fetch();
    if (ball_detector_fetcher_.get()) {
      ball_detected = ball_detector_fetcher_->voltage() > 2.5;
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
        drivetrain_status_fetcher_.Fetch();
        drivetrain_goal_fetcher_.Fetch();
        if (drivetrain_status_fetcher_.get() &&
            drivetrain_goal_fetcher_.get()) {
          const double left_goal = drivetrain_goal_fetcher_->left_goal();
          const double right_goal = drivetrain_goal_fetcher_->right_goal();
          const double left_current =
              drivetrain_status_fetcher_->estimated_left_position();
          const double right_current =
              drivetrain_status_fetcher_->estimated_right_position();
          const double left_velocity =
              drivetrain_status_fetcher_->estimated_left_velocity();
          const double right_velocity =
              drivetrain_status_fetcher_->estimated_right_velocity();
          if (vision_action_running_ && ::std::abs(last_angle_) < 0.02 &&
              ::std::abs((left_goal - right_goal) -
                         (left_current - right_current)) /
                      dt_config().robot_radius / 2.0 <
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
        auto builder = superstructure_goal_sender_.MakeBuilder();

        ::y2016::control_loops::superstructure::Goal::Builder
            superstructure_builder = builder.MakeBuilder<
                ::y2016::control_loops::superstructure::Goal>();
        superstructure_builder.add_angle_intake(intake_goal_);
        superstructure_builder.add_angle_shoulder(shoulder_goal_);
        superstructure_builder.add_angle_wrist(wrist_goal_);

        superstructure_builder.add_max_angular_velocity_intake(7.0);
        superstructure_builder.add_max_angular_velocity_shoulder(4.0);
        superstructure_builder.add_max_angular_velocity_wrist(10.0);
        if (use_slow_profile) {
          superstructure_builder.add_max_angular_acceleration_intake(10.0);
        } else {
          superstructure_builder.add_max_angular_acceleration_intake(40.0);
        }
        superstructure_builder.add_max_angular_acceleration_shoulder(10.0);
        if (shoulder_goal_ > 1.0) {
          superstructure_builder.add_max_angular_acceleration_wrist(45.0);
        } else {
          superstructure_builder.add_max_angular_acceleration_wrist(25.0);
        }

        // Granny mode
        /*
        superstructure_builder.add_max_angular_velocity_intake(0.2);
        superstructure_builder.add_max_angular_velocity_shoulder(0.2);
        superstructure_builder.add_max_angular_velocity_wrist(0.2);
        superstructure_builder.add_max_angular_acceleration_intake(1.0);
        superstructure_builder.add_max_angular_acceleration_shoulder(1.0);
        superstructure_builder.add_max_angular_acceleration_wrist(1.0);
        */
        if (is_intaking_) {
          superstructure_builder.add_voltage_top_rollers(12.0);
          superstructure_builder.add_voltage_bottom_rollers(12.0);
        } else if (is_outtaking_) {
          superstructure_builder.add_voltage_top_rollers(-12.0);
          superstructure_builder.add_voltage_bottom_rollers(-7.0);
        } else {
          superstructure_builder.add_voltage_top_rollers(0.0);
          superstructure_builder.add_voltage_bottom_rollers(0.0);
        }

        superstructure_builder.add_traverse_unlatched(traverse_unlatched_);
        superstructure_builder.add_unfold_climber(false);
        superstructure_builder.add_voltage_climber(voltage_climber);
        superstructure_builder.add_traverse_down(traverse_down_);
        superstructure_builder.add_force_intake(true);

        if (!builder.Send(superstructure_builder.Finish())) {
          AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
        } else {
          AOS_LOG(DEBUG, "sending goals: intake: %f, shoulder: %f, wrist: %f\n",
                  intake_goal_, shoulder_goal_, wrist_goal_);
        }
      }

      {
        auto builder = shooter_goal_sender_.MakeBuilder();
        y2016::control_loops::shooter::Goal::Builder shooter_builder =
            builder.MakeBuilder<y2016::control_loops::shooter::Goal>();
        shooter_builder.add_angular_velocity(shooter_velocity_);
        shooter_builder.add_clamp_open(is_intaking_ || is_outtaking_);
        shooter_builder.add_push_to_shooter(fire_);
        shooter_builder.add_force_lights_on(force_lights_on);
        shooter_builder.add_shooting_forwards(wrist_goal_ > 0);

        if (!builder.Send(shooter_builder.Finish())) {
          AOS_LOG(ERROR, "Sending shooter goal failed.\n");
        }
      }
    }
  }

 private:
  ::aos::Fetcher<::y2016::vision::VisionStatus> vision_status_fetcher_;
  ::aos::Fetcher<::y2016::sensors::BallDetector> ball_detector_fetcher_;
  ::aos::Sender<::y2016::control_loops::shooter::Goal>
      shooter_goal_sender_;
  ::aos::Fetcher<::y2016::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  ::aos::Sender<::y2016::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;

  // Whatever these are set to are our default goals to send out after zeroing.
  double intake_goal_;
  double shoulder_goal_;
  double wrist_goal_;
  double shooter_velocity_ = 0.0;

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

  bool is_expanding_ = false;

  actors::VisionAlignActor::Factory vision_align_action_factory_;
  actors::SuperstructureActor::Factory superstructure_action_factory_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::std::chrono::milliseconds(200), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2016

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2016::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
