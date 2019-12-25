#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <chrono>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "aos/vision/events/udp.h"
#include "external/com_google_protobuf/src/google/protobuf/stubs/stringprintf.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"

#include "y2019/camera_log_generated.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/target_selector_generated.h"
#include "y2019/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2019/control_loops/superstructure/superstructure_position_generated.h"
#include "y2019/control_loops/superstructure/superstructure_status_generated.h"
#include "y2019/vision.pb.h"

using aos::events::ProtoTXUdpSocket;
using aos::input::driver_station::ButtonLocation;
using aos::input::driver_station::ControlBit;
using aos::input::driver_station::JoystickAxis;
using aos::input::driver_station::POVLocation;
using frc971::CreateProfileParameters;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::drivetrain::LocalizerControl;
using y2019::control_loops::superstructure::SuctionGoal;
using y2019::control_loops::superstructure::CreateSuctionGoal;

namespace chrono = ::std::chrono;

namespace y2019 {
namespace input {
namespace joysticks {

namespace superstructure = y2019::control_loops::superstructure;

using google::protobuf::StringPrintf;

struct ElevatorWristPosition {
  double elevator;
  double wrist;
};

const ButtonLocation kSuctionBall(4, 2);
const ButtonLocation kSuctionHatch(3, 15);
const ButtonLocation kDeployStilt(4, 1);
const ButtonLocation kHalfStilt(4, 3);
const ButtonLocation kFallOver(3, 16);

const ButtonLocation kRocketForwardLower(5, 1);
const ButtonLocation kRocketForwardMiddle(5, 2);
const ButtonLocation kRocketForwardUpper(5, 4);
const ButtonLocation kCargoForward(5, 3);

const POVLocation kRocketBackwardUnpressed(5, -1);
const POVLocation kRocketBackwardLower(5, 180);
const POVLocation kRocketBackwardMiddle(5, 90);
const POVLocation kRocketBackwardUpper(5, 0);
const POVLocation kCargoBackward(5, 270);

const ButtonLocation kPanelSwitch(5, 7);
const ButtonLocation kCargoSwitch(5, 8);

const ButtonLocation kBallHPIntakeForward(5, 6);
const ButtonLocation kBallHPIntakeBackward(5, 5);
const JoystickAxis kBallOutake(5, 3);
const JoystickAxis kBallIntake(5, 4);

const ButtonLocation kPanelHPIntakeForward(5, 6);
const ButtonLocation kPanelHPIntakeBackward(5, 5);

const ButtonLocation kRelease(2, 4);
// Reuse quickturn for the cancel button.
const ButtonLocation kCancelAutoMode(2, 3);
const ButtonLocation kReleaseButtonBoard(3, 4);
const ButtonLocation kResetLocalizerLeftForwards(3, 10);
const ButtonLocation kResetLocalizerLeftBackwards(3, 9);

const ButtonLocation kResetLocalizerRightForwards(3, 8);
const ButtonLocation kResetLocalizerRightBackwards(3, 7);

const ButtonLocation kResetLocalizerLeft(3, 11);
const ButtonLocation kResetLocalizerRight(3, 13);

const ButtonLocation kNearCargoHint(3, 3);
const ButtonLocation kMidCargoHint(3, 5);
const ButtonLocation kFarCargoHint(3, 6);

const JoystickAxis kCargoSelectorY(5, 6);
const JoystickAxis kCargoSelectorX(5, 5);

const ButtonLocation kCameraLog(3, 14);

const ElevatorWristPosition kStowPos{0.36, 0.0};
const ElevatorWristPosition kClimbPos{0.0, M_PI / 4.0};

const ElevatorWristPosition kPanelHPIntakeForwrdPos{0.01, M_PI / 2.0};
const ElevatorWristPosition kPanelHPIntakeBackwardPos{0.015, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardLowerPos{0.0, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardLowerPos{0.0, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardMiddlePos{0.75, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardMiddlePos{0.78, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardUpperPos{1.51, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardUpperPos{1.50, -M_PI / 2.0};

const ElevatorWristPosition kPanelCargoForwardPos{0.0, M_PI / 2.0};
const ElevatorWristPosition kPanelCargoBackwardPos{0.0, -M_PI / 2.0};

const ElevatorWristPosition kBallForwardLowerPos{0.21, 1.27};
const ElevatorWristPosition kBallBackwardLowerPos{0.43, -1.99};

const ElevatorWristPosition kBallForwardMiddlePos{0.925, 1.21};
const ElevatorWristPosition kBallBackwardMiddlePos{1.19, -1.98};

const ElevatorWristPosition kBallForwardUpperPos{1.51, 0.961};
const ElevatorWristPosition kBallBackwardUpperPos{1.44, -1.217};

const ElevatorWristPosition kBallCargoForwardPos{0.59, 1.2};
const ElevatorWristPosition kBallCargoBackwardPos{0.868265, -2.1};

const ElevatorWristPosition kBallHPIntakeForwardPos{0.55, 1.097};
const ElevatorWristPosition kBallHPIntakeBackwardPos{0.89, -2.018};

const ElevatorWristPosition kBallIntakePos{0.309, 2.13};

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2019::control_loops::drivetrain::GetDrivetrainConfig(),
            ::aos::input::DrivetrainInputReader::InputType::kPistol,
            {.run_teleop_in_auto = true,
             .cancel_auto_button = kCancelAutoMode}),
        target_selector_hint_sender_(
            event_loop->MakeSender<
                ::y2019::control_loops::drivetrain::TargetSelectorHint>(
                "/drivetrain")),
        localizer_control_sender_(
            event_loop->MakeSender<LocalizerControl>("/drivetrain")),
        camera_log_sender_(
            event_loop->MakeSender<::y2019::CameraLog>("/camera")),
        superstructure_goal_fetcher_(
            event_loop->MakeFetcher<superstructure::Goal>("/superstructure")),
        superstructure_goal_sender_(
            event_loop->MakeSender<superstructure::Goal>("/superstructure")),
        superstructure_position_fetcher_(
            event_loop->MakeFetcher<superstructure::Position>(
                "/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<superstructure::Status>(
                "/superstructure")) {
    const uint16_t team = ::aos::network::GetTeamNumber();
    superstructure_goal_fetcher_.Fetch();
    if (superstructure_goal_fetcher_.get()) {
      grab_piece_ = superstructure_goal_fetcher_->has_suction()
                        ? superstructure_goal_fetcher_->suction()->grab_piece()
                        : false;
      switch_ball_ =
          superstructure_goal_fetcher_->has_suction()
              ? (superstructure_goal_fetcher_->suction()->gamepiece_mode() == 0)
              : true;
    }
    video_tx_.reset(new ProtoTXUdpSocket<VisionControl>(
        StringPrintf("10.%d.%d.179", team / 100, team % 100), 5000));
  }

  void AutoEnded() override {
    AOS_LOG(INFO, "Auto ended, assuming disc and have piece\n");
    grab_piece_ = true;
    switch_ball_ = false;
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) override {
    ::aos::monotonic_clock::time_point monotonic_now =
        ::aos::monotonic_clock::now();
    superstructure_position_fetcher_.Fetch();
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get() ||
        !superstructure_position_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status or position packet.\n");
      return;
    }

    CHECK(superstructure_status_fetcher_->has_stilts());

    if (!superstructure_status_fetcher_->has_piece()) {
      last_not_has_piece_ = monotonic_now;
    }

    auto main_superstructure_goal_builder =
        superstructure_goal_sender_.MakeBuilder();

    flatbuffers::Offset<superstructure::Goal> superstructure_goal_offset;

    {
      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          elevator_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *main_superstructure_goal_builder.fbb(), 0.0,
              CreateProfileParameters(*main_superstructure_goal_builder.fbb(),
                                      0.0, 0.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          intake_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *main_superstructure_goal_builder.fbb(), -1.2);

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *main_superstructure_goal_builder.fbb(), 0.0,
              CreateProfileParameters(*main_superstructure_goal_builder.fbb(),
                                      0.0, 0.0));

      flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
          stilts_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *main_superstructure_goal_builder.fbb(), 0.0,
              CreateProfileParameters(*main_superstructure_goal_builder.fbb(),
                                      0.0, 0.0));

      flatbuffers::Offset<SuctionGoal> suction_offset =
          CreateSuctionGoal(*main_superstructure_goal_builder.fbb(), false, 0);

      superstructure::Goal::Builder superstructure_goal_builder =
          main_superstructure_goal_builder.MakeBuilder<superstructure::Goal>();

      superstructure_goal_builder.add_elevator(elevator_offset);
      superstructure_goal_builder.add_intake(intake_offset);
      superstructure_goal_builder.add_wrist(wrist_offset);
      superstructure_goal_builder.add_stilts(stilts_offset);
      superstructure_goal_builder.add_suction(suction_offset);
      superstructure_goal_builder.add_roller_voltage(0.0);

      superstructure_goal_offset = superstructure_goal_builder.Finish();
    }
    superstructure::Goal *mutable_superstructure_goal = CHECK_NOTNULL(
        GetMutableTemporaryPointer(*main_superstructure_goal_builder.fbb(),
                                   superstructure_goal_offset));

    {
      auto builder = target_selector_hint_sender_.MakeBuilder();
      control_loops::drivetrain::TargetSelectorHint::Builder
          target_selector_hint_builder =
              builder
                  .MakeBuilder<control_loops::drivetrain::TargetSelectorHint>();
      if (data.IsPressed(kNearCargoHint)) {
        target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::NEAR_SHIP);
      } else if (data.IsPressed(kMidCargoHint)) {
        target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::MID_SHIP);
      } else if (data.IsPressed(kFarCargoHint)) {
        target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::FAR_SHIP);
      } else {
        const double cargo_joy_y = data.GetAxis(kCargoSelectorY);
        const double cargo_joy_x = data.GetAxis(kCargoSelectorX);
        if (cargo_joy_y > 0.5) {
          target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::NEAR_SHIP);
        } else if (cargo_joy_y < -0.5) {
          target_selector_hint_builder.add_suggested_target(
              control_loops::drivetrain::SelectionHint::FAR_SHIP);
        } else if (::std::abs(cargo_joy_x) > 0.5) {
          target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::MID_SHIP);
        } else {
          target_selector_hint_builder.add_suggested_target(
            control_loops::drivetrain::SelectionHint::NONE);
        }
      }
      if (!builder.Send(target_selector_hint_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to send target selector hint.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeft)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the left feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.6);
      localizer_control_builder.add_y(3.4);
      localizer_control_builder.add_keep_current_theta(true);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRight)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the right feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.6);
      localizer_control_builder.add_y(-3.4);
      localizer_control_builder.add_keep_current_theta(true);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftForwards)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the left feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.4);
      localizer_control_builder.add_y(3.4);
      localizer_control_builder.add_theta(0.0);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftBackwards)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the left feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.4);
      localizer_control_builder.add_y(3.4);
      localizer_control_builder.add_theta(M_PI);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightForwards)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the right feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.4);
      localizer_control_builder.add_y(-3.4);
      localizer_control_builder.add_theta(0.0);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightBackwards)) {
      auto builder = localizer_control_sender_.MakeBuilder();
      // Start at the right feeder station.
      LocalizerControl::Builder localizer_control_builder =
          builder.MakeBuilder<LocalizerControl>();
      localizer_control_builder.add_x(0.4);
      localizer_control_builder.add_y(-3.4);
      localizer_control_builder.add_theta(M_PI);

      if (!builder.Send(localizer_control_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kRelease) &&
        monotonic_now >
            last_release_button_press_ + ::std::chrono::milliseconds(500)) {
      if (superstructure_status_fetcher_->has_piece()) {
        release_mode_ = ReleaseButtonMode::kRelease;
      } else {
        release_mode_ = ReleaseButtonMode::kBallIntake;
      }
    }

    if (data.IsPressed(kRelease)) {
      last_release_button_press_ = monotonic_now;
    }

    AOS_LOG(INFO, "has_piece: %d\n",
            superstructure_status_fetcher_->has_piece());
    if (data.IsPressed(kSuctionBall)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kSuctionHatch)) {
      grab_piece_ = true;
    } else if ((release_mode_ == ReleaseButtonMode::kRelease &&
                data.IsPressed(kRelease)) ||
               data.IsPressed(kReleaseButtonBoard) ||
               !superstructure_status_fetcher_->has_piece()) {
      grab_piece_ = false;
      AOS_LOG(INFO, "releasing due to other thing\n");
    }

    if (data.IsPressed(kRocketBackwardUnpressed)) {
      elevator_wrist_pos_ = kStowPos;
    }
    mutable_superstructure_goal->mutable_intake()->mutate_unsafe_goal(-1.2);
    mutable_superstructure_goal->mutate_roller_voltage(0.0);

    const bool kDoBallIntake =
        (!climbed_ && release_mode_ == ReleaseButtonMode::kBallIntake &&
         data.IsPressed(kRelease)) ||
        data.GetAxis(kBallIntake) > 0.9;
    const bool kDoBallOutake = data.GetAxis(kBallOutake) > 0.9;

    if (data.IsPressed(kPanelSwitch)) {
      switch_ball_ = false;
    } else if (data.IsPressed(kCargoSwitch)) {
      switch_ball_ = true;
    }

    if (switch_ball_) {
      if (superstructure_status_fetcher_->has_piece()) {
        mutable_superstructure_goal->mutable_wrist()
            ->mutable_profile_params()
            ->mutate_max_acceleration(20);
      }

      // Go to intake position and apply vacuum
      if (data.IsPressed(kBallHPIntakeForward)) {
        grab_piece_ = true;
        elevator_wrist_pos_ = kBallHPIntakeForwardPos;
      } else if (data.IsPressed(kBallHPIntakeBackward)) {
        grab_piece_ = true;
        elevator_wrist_pos_ = kBallHPIntakeBackwardPos;
      }

      // Go to elevator/wrist position. Overrides intake position if pressed so
      // we can re-grab the ball.
      if (data.IsPressed(kRocketForwardLower)) {
        elevator_wrist_pos_ = kBallForwardLowerPos;
      } else if (data.IsPressed(kRocketBackwardLower)) {
        elevator_wrist_pos_ = kBallBackwardLowerPos;
      } else if (data.IsPressed(kRocketForwardMiddle)) {
        elevator_wrist_pos_ = kBallForwardMiddlePos;
      } else if (data.IsPressed(kRocketBackwardMiddle)) {
        elevator_wrist_pos_ = kBallBackwardMiddlePos;
      } else if (data.IsPressed(kRocketForwardUpper)) {
        elevator_wrist_pos_ = kBallForwardUpperPos;
      } else if (data.IsPressed(kRocketBackwardUpper)) {
        elevator_wrist_pos_ = kBallBackwardUpperPos;
      } else if (data.IsPressed(kCargoForward)) {
        elevator_wrist_pos_ = kBallCargoForwardPos;
      } else if (data.IsPressed(kCargoBackward)) {
        elevator_wrist_pos_ = kBallCargoBackwardPos;
      }
    } else {
      if (data.IsPressed(kPanelHPIntakeForward)) {
        grab_piece_ = true;
        elevator_wrist_pos_ = kPanelHPIntakeForwrdPos;
      } else if (data.IsPressed(kPanelHPIntakeBackward)) {
        grab_piece_ = true;
        elevator_wrist_pos_ = kPanelHPIntakeBackwardPos;
      }

      // Go to elevator/wrist position. Overrides intake position if pressed so
      // we can re-grab the panel.
      if (data.IsPressed(kRocketForwardLower)) {
        elevator_wrist_pos_ = kPanelForwardLowerPos;
      } else if (data.IsPressed(kRocketBackwardLower)) {
        elevator_wrist_pos_ = kPanelBackwardLowerPos;
      } else if (data.IsPressed(kRocketForwardMiddle)) {
        elevator_wrist_pos_ = kPanelForwardMiddlePos;
      } else if (data.IsPressed(kRocketBackwardMiddle)) {
        elevator_wrist_pos_ = kPanelBackwardMiddlePos;
      } else if (data.IsPressed(kRocketForwardUpper)) {
        elevator_wrist_pos_ = kPanelForwardUpperPos;
      } else if (data.IsPressed(kRocketBackwardUpper)) {
        elevator_wrist_pos_ = kPanelBackwardUpperPos;
      } else if (data.IsPressed(kCargoForward)) {
        elevator_wrist_pos_ = kPanelCargoForwardPos;
      } else if (data.IsPressed(kCargoBackward)) {
        elevator_wrist_pos_ = kPanelCargoBackwardPos;
      }
    }

    if (switch_ball_) {
      if (kDoBallOutake ||
          (kDoBallIntake &&
           monotonic_now < last_not_has_piece_ + chrono::milliseconds(200))) {
        mutable_superstructure_goal->mutable_intake()->mutate_unsafe_goal(0.83);
      }

      if (kDoBallIntake && !superstructure_status_fetcher_->has_piece()) {
        elevator_wrist_pos_ = kBallIntakePos;
        mutable_superstructure_goal->mutate_roller_voltage(9.0);
        grab_piece_ = true;
      } else {
        if (kDoBallOutake) {
          mutable_superstructure_goal->mutate_roller_voltage(-6.0);
        } else {
          mutable_superstructure_goal->mutate_roller_voltage(0.0);
        }
      }
    }

    constexpr double kDeployStiltPosition = 0.5;

    if (data.IsPressed(kFallOver)) {
      mutable_superstructure_goal->mutable_stilts()->mutate_unsafe_goal(0.71);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_velocity(0.65);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_acceleration(1.25);
    } else if (data.IsPressed(kHalfStilt)) {
      was_above_ = false;
      mutable_superstructure_goal->mutable_stilts()->mutate_unsafe_goal ( 0.345);
      mutable_superstructure_goal->mutable_stilts()->mutable_profile_params()->mutate_max_velocity ( 0.65);
    } else if (data.IsPressed(kDeployStilt) || was_above_) {
      mutable_superstructure_goal->mutable_stilts()->mutate_unsafe_goal(
          kDeployStiltPosition);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_velocity(0.65);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_acceleration(2.0);
    } else {
      mutable_superstructure_goal->mutable_stilts()->mutate_unsafe_goal(0.005);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_velocity(0.65);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_acceleration(2.0);
    }

    if (superstructure_status_fetcher_->stilts()->position() > 0.1) {
      elevator_wrist_pos_ = kClimbPos;
      climbed_ = true;
      mutable_superstructure_goal->mutable_wrist()
          ->mutable_profile_params()
          ->mutate_max_acceleration(10);
      mutable_superstructure_goal->mutable_elevator()
          ->mutable_profile_params()
          ->mutate_max_acceleration(6);
    }

    // If we've been asked to go above deploy and made it up that high, latch
    // was_above.
    if (mutable_superstructure_goal->stilts()->unsafe_goal() >
            kDeployStiltPosition &&
        superstructure_status_fetcher_->stilts()->position() >=
            kDeployStiltPosition) {
      was_above_ = true;
    } else if ((superstructure_position_fetcher_->platform_left_detect() &&
                superstructure_position_fetcher_->platform_right_detect()) &&
               !data.IsPressed(kDeployStilt) && !data.IsPressed(kFallOver)) {
      was_above_ = false;
    }

    if (superstructure_status_fetcher_->stilts()->position() >
            kDeployStiltPosition &&
        mutable_superstructure_goal->stilts()->unsafe_goal() ==
            kDeployStiltPosition) {
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_velocity(0.30);
      mutable_superstructure_goal->mutable_stilts()
          ->mutable_profile_params()
          ->mutate_max_acceleration(0.75);
    }

    if ((release_mode_ == ReleaseButtonMode::kRelease &&
         data.IsPressed(kRelease)) ||
        data.IsPressed(kReleaseButtonBoard)) {
      grab_piece_ = false;
      AOS_LOG(INFO, "Releasing due to button\n");
    }

    if (switch_ball_) {
      CHECK_NOTNULL(mutable_superstructure_goal->mutable_suction())
          ->mutate_gamepiece_mode(0);
    } else {
      CHECK_NOTNULL(mutable_superstructure_goal->mutable_suction())
          ->mutate_gamepiece_mode(1);
    }

    vision_control_.set_flip_image(elevator_wrist_pos_.wrist < 0);

    CHECK_NOTNULL(mutable_superstructure_goal->mutable_suction())
        ->mutate_grab_piece(grab_piece_);

    mutable_superstructure_goal->mutable_elevator()->mutate_unsafe_goal(
        elevator_wrist_pos_.elevator);
    mutable_superstructure_goal->mutable_wrist()->mutate_unsafe_goal(
        elevator_wrist_pos_.wrist);

    if (!main_superstructure_goal_builder.Send(superstructure_goal_offset)) {
      AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
    }

    if (monotonic_now >
        last_vision_control_ + ::std::chrono::milliseconds(50)) {
      video_tx_->Send(vision_control_);
      last_vision_control_ = monotonic_now;
    }

    {
      auto builder = camera_log_sender_.MakeBuilder();
      builder.Send(CreateCameraLog(*builder.fbb(), data.IsPressed(kCameraLog)));
    }
  }

 private:
  ::aos::Sender<::y2019::control_loops::drivetrain::TargetSelectorHint>
      target_selector_hint_sender_;

  ::aos::Sender<LocalizerControl> localizer_control_sender_;

  ::aos::Sender<::y2019::CameraLog> camera_log_sender_;

  ::aos::Fetcher<superstructure::Goal> superstructure_goal_fetcher_;

  ::aos::Sender<superstructure::Goal> superstructure_goal_sender_;

  ::aos::Fetcher<superstructure::Position> superstructure_position_fetcher_;
  ::aos::Fetcher<superstructure::Status> superstructure_status_fetcher_;

  // Bool to track if we've been above the deploy position.  Once this bool is
  // set, don't let the stilts retract until we see the platform.
  bool was_above_ = false;

  // Current goals here.
  ElevatorWristPosition elevator_wrist_pos_ = kStowPos;
  bool grab_piece_ = false;

  bool switch_ball_ = false;

  bool climbed_ = false;

  enum class ReleaseButtonMode {
    kBallIntake,
    kRelease,
  };

  ReleaseButtonMode release_mode_ = ReleaseButtonMode::kRelease;
  aos::monotonic_clock::time_point last_release_button_press_ =
      aos::monotonic_clock::min_time;

  VisionControl vision_control_;
  ::std::unique_ptr<ProtoTXUdpSocket<VisionControl>> video_tx_;
  ::aos::monotonic_clock::time_point last_vision_control_ =
      ::aos::monotonic_clock::time_point::min();

  // Time at which we last did not have a game piece.
  ::aos::monotonic_clock::time_point last_not_has_piece_ =
      ::aos::monotonic_clock::time_point::min();
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2019

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2019::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
