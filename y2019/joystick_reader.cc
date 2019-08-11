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
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "aos/vision/events/udp.h"
#include "external/com_google_protobuf/src/google/protobuf/stubs/stringprintf.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"

#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/target_selector.q.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"
#include "y2019/status_light.q.h"
#include "y2019/vision.pb.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;
using ::aos::events::ProtoTXUdpSocket;

namespace chrono = ::std::chrono;

namespace y2019 {
namespace input {
namespace joysticks {

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
                ".y2019.control_loops.drivetrain.target_selector_hint")),
        localizer_control_sender_(
            event_loop->MakeSender<
                ::frc971::control_loops::drivetrain::LocalizerControl>(
                ".frc971.control_loops.drivetrain.localizer_control")),
        camera_log_sender_(
            event_loop->MakeSender<::y2019::CameraLog>(".y2019.camera_log")),
        superstructure_goal_fetcher_(event_loop->MakeFetcher<
                                     ::y2019::control_loops::superstructure::
                                         SuperstructureQueue::Goal>(
            ".y2019.control_loops.superstructure.superstructure_queue.goal")),
        superstructure_goal_sender_(event_loop->MakeSender<
                                    ::y2019::control_loops::superstructure::
                                        SuperstructureQueue::Goal>(
            ".y2019.control_loops.superstructure.superstructure_queue.goal")),
        superstructure_position_fetcher_(
            event_loop->MakeFetcher<::y2019::control_loops::superstructure::
                                        SuperstructureQueue::Position>(
                ".y2019.control_loops.superstructure.superstructure_queue."
                "position")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<::y2019::control_loops::superstructure::
                                        SuperstructureQueue::Status>(
                ".y2019.control_loops.superstructure.superstructure_queue."
                "status")) {
    const uint16_t team = ::aos::network::GetTeamNumber();
    superstructure_goal_fetcher_.Fetch();
    if (superstructure_goal_fetcher_.get()) {
      grab_piece_ = superstructure_goal_fetcher_->suction.grab_piece;
      switch_ball_ = superstructure_goal_fetcher_->suction.gamepiece_mode == 0;
    }
    video_tx_.reset(new ProtoTXUdpSocket<VisionControl>(
        StringPrintf("10.%d.%d.179", team / 100, team % 100), 5000));
  }

  void AutoEnded() override {
    LOG(INFO, "Auto ended, assuming disc and have piece\n");
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
      LOG(ERROR, "Got no superstructure status or position packet.\n");
      return;
    }

    if (!superstructure_status_fetcher_->has_piece) {
      last_not_has_piece_ = monotonic_now;
    }

    auto new_superstructure_goal = superstructure_goal_sender_.MakeMessage();

    {
      auto target_hint = target_selector_hint_sender_.MakeMessage();
      if (data.IsPressed(kNearCargoHint)) {
        target_hint->suggested_target = 1;
      } else if (data.IsPressed(kMidCargoHint)) {
        target_hint->suggested_target = 2;
      } else if (data.IsPressed(kFarCargoHint)) {
        target_hint->suggested_target = 3;
      } else {
        const double cargo_joy_y = data.GetAxis(kCargoSelectorY);
        const double cargo_joy_x = data.GetAxis(kCargoSelectorX);
        if (cargo_joy_y > 0.5) {
          target_hint->suggested_target = 1;
        } else if (cargo_joy_y < -0.5) {
          target_hint->suggested_target = 3;
        } else if (::std::abs(cargo_joy_x) > 0.5) {
          target_hint->suggested_target = 2;
        } else {
          target_hint->suggested_target = 0;
        }
      }
      if (!target_hint.Send()) {
        LOG(ERROR, "Failed to send target selector hint.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeft)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.6;
      localizer_resetter->y = 3.4;
      localizer_resetter->keep_current_theta = true;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRight)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.6;
      localizer_resetter->y = -3.4;
      localizer_resetter->keep_current_theta = true;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftForwards)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = 3.4;
      localizer_resetter->theta = 0.0;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftBackwards)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = 3.4;
      localizer_resetter->theta = M_PI;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightForwards)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the right feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = -3.4;
      localizer_resetter->theta = 0.0;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightBackwards)) {
      auto localizer_resetter = localizer_control_sender_.MakeMessage();
      // Start at the right feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = -3.4;
      localizer_resetter->theta = M_PI;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kRelease) &&
        monotonic_now >
            last_release_button_press_ + ::std::chrono::milliseconds(500)) {
      if (superstructure_status_fetcher_->has_piece) {
        release_mode_ = ReleaseButtonMode::kRelease;
      } else {
        release_mode_ = ReleaseButtonMode::kBallIntake;
      }
    }

    if (data.IsPressed(kRelease)) {
      last_release_button_press_ = monotonic_now;
    }

    LOG(INFO, "has_piece: %d\n", superstructure_status_fetcher_->has_piece);
    if (data.IsPressed(kSuctionBall)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kSuctionHatch)) {
      grab_piece_ = true;
    } else if ((release_mode_ == ReleaseButtonMode::kRelease &&
                data.IsPressed(kRelease)) ||
               data.IsPressed(kReleaseButtonBoard) ||
               !superstructure_status_fetcher_->has_piece) {
      grab_piece_ = false;
      LOG(INFO, "releasing due to other thing\n");
    }

    if (data.IsPressed(kRocketBackwardUnpressed)) {
      elevator_wrist_pos_ = kStowPos;
    }
    new_superstructure_goal->intake.unsafe_goal = -1.2;
    new_superstructure_goal->roller_voltage = 0.0;

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
      if (superstructure_status_fetcher_->has_piece) {
        new_superstructure_goal->wrist.profile_params.max_acceleration = 20;
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
        new_superstructure_goal->intake.unsafe_goal = 0.83;
      }

      if (kDoBallIntake && !superstructure_status_fetcher_->has_piece) {
        elevator_wrist_pos_ = kBallIntakePos;
        new_superstructure_goal->roller_voltage = 9.0;
        grab_piece_ = true;
      } else {
        if (kDoBallOutake) {
          new_superstructure_goal->roller_voltage = -6.0;
        } else {
          new_superstructure_goal->roller_voltage = 0.0;
        }
      }
    }

    constexpr double kDeployStiltPosition = 0.5;

    if (data.IsPressed(kFallOver)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.71;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 1.25;
    } else if (data.IsPressed(kHalfStilt)) {
      was_above_ = false;
      new_superstructure_goal->stilts.unsafe_goal = 0.345;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
    } else if (data.IsPressed(kDeployStilt) || was_above_) {
      new_superstructure_goal->stilts.unsafe_goal = kDeployStiltPosition;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    } else {
      new_superstructure_goal->stilts.unsafe_goal = 0.005;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    }

    if (superstructure_status_fetcher_->stilts.position > 0.1) {
      elevator_wrist_pos_ = kClimbPos;
      climbed_ = true;
      new_superstructure_goal->wrist.profile_params.max_acceleration = 10;
      new_superstructure_goal->elevator.profile_params.max_acceleration = 6;
    }

    // If we've been asked to go above deploy and made it up that high, latch
    // was_above.
    if (new_superstructure_goal->stilts.unsafe_goal > kDeployStiltPosition &&
        superstructure_status_fetcher_->stilts.position >=
            kDeployStiltPosition) {
      was_above_ = true;
    } else if ((superstructure_position_fetcher_->platform_left_detect &&
                superstructure_position_fetcher_->platform_right_detect) &&
               !data.IsPressed(kDeployStilt) && !data.IsPressed(kFallOver)) {
      was_above_ = false;
    }

    if (superstructure_status_fetcher_->stilts.position >
            kDeployStiltPosition &&
        new_superstructure_goal->stilts.unsafe_goal == kDeployStiltPosition) {
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.30;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 0.75;
    }

    if ((release_mode_ == ReleaseButtonMode::kRelease &&
         data.IsPressed(kRelease)) ||
        data.IsPressed(kReleaseButtonBoard)) {
      grab_piece_ = false;
      LOG(INFO, "Releasing due to button\n");
    }

    if (switch_ball_) {
      new_superstructure_goal->suction.gamepiece_mode = 0;
    } else {
      new_superstructure_goal->suction.gamepiece_mode = 1;
    }

    vision_control_.set_flip_image(elevator_wrist_pos_.wrist < 0);

    new_superstructure_goal->suction.grab_piece = grab_piece_;

    new_superstructure_goal->elevator.unsafe_goal =
        elevator_wrist_pos_.elevator;
    new_superstructure_goal->wrist.unsafe_goal = elevator_wrist_pos_.wrist;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }

    if (monotonic_now >
        last_vision_control_ + ::std::chrono::milliseconds(50)) {
      video_tx_->Send(vision_control_);
      last_vision_control_ = monotonic_now;
    }

    {
      auto camera_log_message = camera_log_sender_.MakeMessage();
      camera_log_message->log = data.IsPressed(kCameraLog);
      LOG_STRUCT(DEBUG, "camera_log", *camera_log_message);
      camera_log_message.Send();
    }
  }

 private:
  ::aos::Sender<::y2019::control_loops::drivetrain::TargetSelectorHint>
      target_selector_hint_sender_;

  ::aos::Sender<::frc971::control_loops::drivetrain::LocalizerControl>
      localizer_control_sender_;

  ::aos::Sender<::y2019::CameraLog> camera_log_sender_;

  ::aos::Fetcher<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Goal>
      superstructure_goal_fetcher_;

  ::aos::Sender<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Goal>
      superstructure_goal_sender_;

  ::aos::Fetcher<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Position>
      superstructure_position_fetcher_;
  ::aos::Fetcher<
      ::y2019::control_loops::superstructure::SuperstructureQueue::Status>
      superstructure_status_fetcher_;


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

  ::aos::ShmEventLoop event_loop;
  ::y2019::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
