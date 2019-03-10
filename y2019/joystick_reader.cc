#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/actions/actions.h"
#include "aos/init.h"
#include "aos/input/action_joystick_input.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/drivetrain_input.h"
#include "aos/input/joystick_input.h"
#include "aos/logging/logging.h"
#include "aos/logging/logging.h"
#include "aos/util/log_interval.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/autonomous/base_autonomous_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/superstructure/superstructure.q.h"
#include "y2019/status_light.q.h"

using ::y2019::control_loops::superstructure::superstructure_queue;
using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace y2019 {
namespace input {
namespace joysticks {

const ButtonLocation kSuctionBall(3, 13);
const ButtonLocation kSuctionHatch(3, 12);
const ButtonLocation kDeployStilt(3, 8);
const ButtonLocation kHalfStilt(3, 6);
const ButtonLocation kFallOver(3, 9);

struct ElevatorWristPosition {
  double elevator;
  double wrist;
};

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

const ButtonLocation kAutoPanel(3, 10);
const ButtonLocation kAutoPanelIntermediate(4, 6);

const ElevatorWristPosition kAutoPanelPos{0.0, -M_PI / 2.0};
const ElevatorWristPosition kAutoPanelIntermediatePos{0.34, -M_PI / 2.0};

const ElevatorWristPosition kStowPos{0.36, 0.0};

const ElevatorWristPosition kPanelHPIntakeForwrdPos{0.04, M_PI / 2.0};
const ElevatorWristPosition kPanelHPIntakeBackwardPos{0.05, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardLowerPos{0.0, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardLowerPos{0.0, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardMiddlePos{0.75, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardMiddlePos{0.78, -M_PI / 2.0};

const ElevatorWristPosition kPanelForwardUpperPos{1.51, M_PI / 2.0};
const ElevatorWristPosition kPanelBackwardUpperPos{1.50, -M_PI / 2.0};

const ElevatorWristPosition kPanelCargoForwardPos{0.0, M_PI / 2.0};
const ElevatorWristPosition kPanelCargoBackwardPos{0.0, -M_PI / 2.0};

const ElevatorWristPosition kBallForwardLowerPos{0.46, M_PI / 2.0};
const ElevatorWristPosition kBallBackwardLowerPos{0.15, -M_PI / 2.0};

const ElevatorWristPosition kBallForwardMiddlePos{1.16, 1.546};
const ElevatorWristPosition kBallBackwardMiddlePos{0.876021, -1.546};

const ElevatorWristPosition kBallForwardUpperPos{1.50, 0.961};
const ElevatorWristPosition kBallBackwardUpperPos{1.41, -1.217};

const ElevatorWristPosition kBallCargoForwardPos{0.699044, 1.353};
const ElevatorWristPosition kBallCargoBackwardPos{0.828265, -1.999};

const ElevatorWristPosition kBallHPIntakeForwardPos{0.55, 1.097};
const ElevatorWristPosition kBallHPIntakeBackwardPos{0.89, -2.018};

const ElevatorWristPosition kBallIntakePos{0.29, 2.14};

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2019::control_loops::drivetrain::GetDrivetrainConfig()) {
    superstructure_queue.goal.FetchLatest();
    if (superstructure_queue.goal.get()) {
      grab_piece_ = superstructure_queue.goal->suction.grab_piece;
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    superstructure_queue.position.FetchLatest();
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get() ||
        !superstructure_queue.position.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    if (data.IsPressed(kSuctionBall)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kSuctionHatch)) {
      grab_piece_ = true;
    } else if (data.IsPressed(kRelease) ||
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

    // TODO(sabina): max height please?
    if (data.IsPressed(kFallOver)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.71;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    } else if (data.IsPressed(kDeployStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.50;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      if (stilts_was_above_) {
        new_superstructure_goal->stilts.profile_params.max_acceleration = 0.75;
      } else {
        new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
      }
    } else if (data.IsPressed(kHalfStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.345;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.65;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 0.75;
    } else {
      new_superstructure_goal->stilts.unsafe_goal = 0.005;
      new_superstructure_goal->stilts.profile_params.max_velocity = 0.25;
      new_superstructure_goal->stilts.profile_params.max_acceleration = 2.0;
    }

    if (superstructure_queue.status->stilts.position > 0.65) {
      stilts_was_above_ = true;
    } else if (superstructure_queue.status->stilts.position < 0.1) {
      stilts_was_above_ = false;
    }

    if (data.IsPressed(kAutoPanel)) {
      elevator_wrist_pos_ = kAutoPanelPos;
    } else if (data.IsPressed(kAutoPanelIntermediate)) {
      elevator_wrist_pos_ = kAutoPanelIntermediatePos;
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
          (kDoBallIntake && !superstructure_queue.status->has_piece)) {
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
          new_superstructure_goal->intake.unsafe_goal = -1.2;
          new_superstructure_goal->roller_voltage = 0.0;
        }
      }
    }

    if (data.IsPressed(kRelease)) {
      grab_piece_ = false;
    }

    if (switch_ball_) {
      new_superstructure_goal->suction.gamepiece_mode = 0;
    } else {
      new_superstructure_goal->suction.gamepiece_mode = 1;
    }

    new_superstructure_goal->suction.grab_piece = grab_piece_;

    new_superstructure_goal->elevator.unsafe_goal =
        elevator_wrist_pos_.elevator;
    new_superstructure_goal->wrist.unsafe_goal = elevator_wrist_pos_.wrist;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  // Current goals here.
  ElevatorWristPosition elevator_wrist_pos_ = kStowPos;
  bool grab_piece_ = false;

  bool switch_ball_ = false;
  bool stilts_was_above_ = false;
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
