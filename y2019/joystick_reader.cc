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

using ::y2019::control_loops::superstructure::superstructure_queue;
using ::y2019::control_loops::drivetrain::target_selector_hint;
using ::frc971::control_loops::drivetrain::localizer_control;
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
const ButtonLocation kReleaseButtonBoard(3, 4);
const ButtonLocation kResetLocalizerLeftForwards(3, 10);
const ButtonLocation kResetLocalizerLeftBackwards(3, 9);

const ButtonLocation kResetLocalizerRightForwards(3, 8);
const ButtonLocation kResetLocalizerRightBackwards(3, 7);

const ButtonLocation kNearCargoHint(3, 3);
const ButtonLocation kMidCargoHint(3, 5);
const ButtonLocation kFarCargoHint(3, 6);

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
            ::y2019::control_loops::drivetrain::GetDrivetrainConfig()) {
    // Set teleop to immediately resume after auto ends for sandstorm mode.
    set_run_teleop_in_auto(true);

    const uint16_t team = ::aos::network::GetTeamNumber();
    superstructure_queue.goal.FetchLatest();
    if (superstructure_queue.goal.get()) {
      grab_piece_ = superstructure_queue.goal->suction.grab_piece;
      switch_ball_ = superstructure_queue.goal->suction.gamepiece_mode == 0;
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
    superstructure_queue.position.FetchLatest();
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get() ||
        !superstructure_queue.position.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    if (!superstructure_queue.status->has_piece) {
      last_not_has_piece_ = monotonic_now;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    {
      auto target_hint = target_selector_hint.MakeMessage();
      if (data.IsPressed(kNearCargoHint)) {
        target_hint->suggested_target = 1;
      } else if (data.IsPressed(kMidCargoHint)) {
        target_hint->suggested_target = 2;
      } else if (data.IsPressed(kFarCargoHint)) {
        target_hint->suggested_target = 3;
      } else {
        target_hint->suggested_target = 0;
      }
      if (!target_hint.Send()) {
        LOG(ERROR, "Failed to send target selector hint.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftForwards)) {
      auto localizer_resetter = localizer_control.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = 3.4;
      localizer_resetter->theta = 0.0;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerLeftBackwards)) {
      auto localizer_resetter = localizer_control.MakeMessage();
      // Start at the left feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = 3.4;
      localizer_resetter->theta = M_PI;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightForwards)) {
      auto localizer_resetter = localizer_control.MakeMessage();
      // Start at the right feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = -3.4;
      localizer_resetter->theta = 0.0;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.PosEdge(kResetLocalizerRightBackwards)) {
      auto localizer_resetter = localizer_control.MakeMessage();
      // Start at the right feeder station.
      localizer_resetter->x = 0.4;
      localizer_resetter->y = -3.4;
      localizer_resetter->theta = M_PI;

      if (!localizer_resetter.Send()) {
        LOG(ERROR, "Failed to reset localizer.\n");
      }
    }

    if (data.IsPressed(kSuctionBall)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kSuctionHatch)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kRelease) ||
               data.IsPressed(kReleaseButtonBoard) ||
               !superstructure_queue.status->has_piece) {
      grab_piece_ = false;
    }

    if (data.IsPressed(kRocketBackwardUnpressed)) {
      elevator_wrist_pos_ = kStowPos;
    }
    new_superstructure_goal->intake.unsafe_goal = -1.2;
    new_superstructure_goal->roller_voltage = 0.0;

    const bool kDoBallIntake = data.GetAxis(kBallIntake) > 0.9;
    const bool kDoBallOutake = data.GetAxis(kBallOutake) > 0.9;

    if (data.IsPressed(kPanelSwitch)) {
      switch_ball_ = false;
    } else if (data.IsPressed(kCargoSwitch)) {
      switch_ball_ = true;
    }

    if (switch_ball_) {
      if (superstructure_queue.status->has_piece) {
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
        new_superstructure_goal->intake.unsafe_goal = 0.959327;
      }

      if (kDoBallIntake && !superstructure_queue.status->has_piece) {
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

    // TODO(sabina): max height please?
    if (data.IsPressed(kFallOver)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.71;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 1.25;
    } else if (data.IsPressed(kDeployStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = kDeployStiltPosition;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    } else if (data.IsPressed(kHalfStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.345;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
    } else {
      new_superstructure_goal->stilts.unsafe_goal = 0.005;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    }

    if (superstructure_queue.status->stilts.position > 0.1) {
      elevator_wrist_pos_ = kClimbPos;
      new_superstructure_goal->wrist.profile_params.max_acceleration = 10;
      new_superstructure_goal->elevator.profile_params.max_acceleration = 6;
    }

    if (superstructure_queue.status->stilts.position > kDeployStiltPosition &&
        new_superstructure_goal->stilts.unsafe_goal == kDeployStiltPosition) {
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.30;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 0.75;
    }

    if (data.IsPressed(kRelease) || data.IsPressed(kReleaseButtonBoard)) {
      grab_piece_ = false;
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
      auto camera_log_message = camera_log.MakeMessage();
      camera_log_message->log = data.IsPressed(kCameraLog);
      LOG_STRUCT(DEBUG, "camera_log", *camera_log_message);
      camera_log_message.Send();
    }
  }

 private:
  uint32_t GetAutonomousMode() override {
    ::frc971::autonomous::auto_mode.FetchLatest();
    if (::frc971::autonomous::auto_mode.get() == nullptr) {
      LOG(WARNING, "no auto mode values\n");
      return 0;
    }
    return ::frc971::autonomous::auto_mode->mode;
  }

  // Current goals here.
  ElevatorWristPosition elevator_wrist_pos_ = kStowPos;
  bool grab_piece_ = false;

  bool switch_ball_ = false;

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
  ::aos::Init(-1);
  ::aos::ShmEventLoop event_loop;
  ::y2019::input::joysticks::Reader reader(&event_loop);
  reader.Run();
  ::aos::Cleanup();
}
