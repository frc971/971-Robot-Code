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

using ::y2019::control_loops::superstructure::superstructure_queue;
using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace y2019 {
namespace input {
namespace joysticks {

// TODO(sabina): update button locations when the board is done
const ButtonLocation kIntakeOut(3, 3);
const ButtonLocation kElevatorDown(0, 0);
const ButtonLocation kElevatorFront1(4, 1);
const ButtonLocation kElevatorFront2(4, 11);
const ButtonLocation kElevatorFront3(4, 9);
const ButtonLocation kElevatorFront4(4, 7);
const ButtonLocation kElevatorFront5(4, 5);

const ButtonLocation kElevatorBack1(3, 14);
const ButtonLocation kElevatorBack2(4, 12);
const ButtonLocation kElevatorBack3(4, 10);
const ButtonLocation kElevatorBack4(4, 8);
const ButtonLocation kElevatorBack5(4, 6);

const ButtonLocation kElevatorIntaking(3, 4);
const ButtonLocation kElevatorIntakingUp(3, 6);
const ButtonLocation kRelease(4, 4);

const ButtonLocation kSuctionBall(3, 13);
const ButtonLocation kSuctionHatch(3, 12);
const ButtonLocation kDeployStilt(3, 8);
const ButtonLocation kFallOver(3, 9);

const ButtonLocation kDiskLoad(0, 0);
const ButtonLocation kDiskRocketMiddle(0, 0);
const ButtonLocation kDiskRocketTop(0, 0);
const ButtonLocation kCargoLoad(0, 0);
const ButtonLocation kCargoBay(0, 0);
const ButtonLocation kCargoRocketBase(0, 0);
const ButtonLocation kCargoRocketMiddle(0, 0);
const ButtonLocation kCargoRocketTop(0, 0);
const ButtonLocation kStow(0, 0);
const ButtonLocation kIntakeExtend(0, 0);
const ButtonLocation kIntake(0, 0);
const ButtonLocation kSpit(0, 0);
const ButtonLocation kCargoSuction(0, 0);
const ButtonLocation kDiskSuction(0, 0);
const ButtonLocation kSuctionOut(0, 0);
const ButtonLocation kRetractStilt(0, 0);
const ButtonLocation kBackwards(0, 0);

const ButtonLocation kWristBackwards(3, 10);
const ButtonLocation kWristForwards(3, 7);

class Reader : public ::aos::input::ActionJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::ActionJoystickInput(
            event_loop,
            ::y2019::control_loops::drivetrain::GetDrivetrainConfig()) {}

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    superstructure_queue.position.FetchLatest();
    superstructure_queue.status.FetchLatest();
    if (!superstructure_queue.status.get() ||
        !superstructure_queue.position.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
      return;
    }

    auto new_superstructure_goal = superstructure_queue.goal.MakeMessage();

    /*
    if (data.IsPressed(kElevatorUp)) {
      elevator_height_ += 0.1;
    } else if (data.IsPressed(kElevatorDown)) {
      elevator_height_ -= 0.1;
    } else if (data.IsPressed(kDiskLoad)) {
      elevator_height_ = 0.48;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kDiskRocketMiddle)) {
      elevator_height_ = 1.19;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kDiskRocketTop)) {
      elevator_height_ = 1.90;
      wrist_angle_ = M_PI;
    }

    // TODO(sabina): do we need an angle here?
    else if (data.IsPressed(kCargoLoad)) {
      elevator_height_ = 1.12;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kCargoBay)) {
      elevator_height_ = 0.0;
      wrist_angle_ = M_PI / 3;
    } else if (data.IsPressed(kCargoRocketBase)) {
      elevator_height_ = 0.7;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kCargoRocketMiddle)) {
      elevator_height_ = 1.41;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kCargoRocketTop)) {
      elevator_height_ = 2.12;
      wrist_angle_ = M_PI;
    } else if (data.IsPressed(kStow)) {
      elevator_height_ = 0.5;
      wrist_angle_ = 0.0;
    } else {
    }
    */

    /*
    // TODO(sabina): get accurate angle.
    if (data.IsPressed(kIntakeExtend)) {
      new_superstructure_goal->intake.unsafe_goal = 0.5;
    } else {
      new_superstructure_goal->intake.unsafe_goal = 0.0;
    }

    if (data.IsPressed(kIntake)) {
      new_superstructure_goal->suction.bottom = true;
      if (superstructure_queue.status->has_piece == false) {
        new_superstructure_goal->roller_voltage = 12.0;
      } else {
        new_superstructure_goal->roller_voltage = 0.0;
      }
    } else if (data.IsPressed(kSpit)) {
      new_superstructure_goal->suction.bottom = false;
      if (superstructure_queue.status->has_piece == false) {
        new_superstructure_goal->roller_voltage = 12.0;
      } else {
        new_superstructure_goal->roller_voltage = 0.0;
      }
    } else {
      new_superstructure_goal->roller_voltage = 0.0;
    }
    */

    // TODO(sabina): decide if we should really have disk suction as its own
    // button
    /*
    if (data.IsPressed(kCargoSuction)) {
      new_superstructure_goal->suction.top = false;
      new_superstructure_goal->suction.bottom = true;
    } else if (data.IsPressed(kDiskSuction)) {
      new_superstructure_goal->suction.top = true;
      new_superstructure_goal->suction.bottom = true;
    } else if (data.IsPressed(kSuctionOut)) {
      new_superstructure_goal->suction.top = true;
      new_superstructure_goal->suction.bottom = true;
    }
    */

    if (data.IsPressed(kSuctionBall)) {
      top_ = false;
      bottom_ = true;
    } else if (data.IsPressed(kSuctionHatch)) {
      top_ = true;
      bottom_ = true;
    } else if (data.IsPressed(kRelease) ||
               !superstructure_queue.status->has_piece) {
      top_ = false;
      bottom_ = false;
    }

    // TODO(sabina): max height please?
    if (data.IsPressed(kFallOver)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.71;
    } else if (data.IsPressed(kDeployStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.50;
    } else {
      new_superstructure_goal->stilts.unsafe_goal = 0.01;
    }

    if (data.IsPressed(kElevatorFront1)) {
      elevator_height_ = 1.5;
    } else if (data.IsPressed(kElevatorFront2)) {
      elevator_height_ = 1.2;
    } else if (data.IsPressed(kElevatorFront3)) {
      elevator_height_ = 0.8;
    } else if (data.IsPressed(kElevatorFront4)) {
      elevator_height_ = 0.3;
    } else if (data.IsPressed(kElevatorFront5)) {
      elevator_height_ = 0.01;
    }

    /*
    if (data.IsPressed(kWristDown)) {
      wrist_angle_ = -M_PI / 3.0;
    } else {
      wrist_angle_ = M_PI / 3.0;
    }
    */
    if (data.IsPressed(kWristBackwards)) {
      // Hatch pannel back
      elevator_height_ = 0.03;
      wrist_angle_ = -M_PI / 2.0;
      Disc();
    } else if (data.IsPressed(kWristForwards)) {
      // Hatch pannel front
      elevator_height_ = 0.03;
      wrist_angle_ = M_PI / 2.0;
      Disc();
    } else if (data.IsPressed(kElevatorFront5)) {
      // Ball front
      Ball();
      elevator_height_ = 0.52;
      wrist_angle_ = 1.1;
    } else if (data.IsPressed(kElevatorBack5)) {
      // Ball back
      elevator_height_ = 0.52;
      wrist_angle_ = -1.1;
    } else if (data.IsPressed(kElevatorFront2)) {
      elevator_height_ = 1.5;
      wrist_angle_ = 0.0;
    } else {
      wrist_angle_ = 0.0;
      elevator_height_ = 0.36;
    }
    //if (data.IsPressed(kElevatorIntaking)) {
    //}
    if (data.IsPressed(kIntakeOut) && !superstructure_queue.status->has_piece) {
      elevator_height_ = 0.29;
      wrist_angle_ = 2.14;
      new_superstructure_goal->intake.unsafe_goal = 0.52;
      if (data.IsPressed(kElevatorIntaking)) {
        new_superstructure_goal->roller_voltage = 6.0;
      } else {
        new_superstructure_goal->roller_voltage = 0.0;
      }
      Ball();
    } else {
      new_superstructure_goal->intake.unsafe_goal = -1.2;
      new_superstructure_goal->roller_voltage = 0.0;
    }

    if (data.IsPressed(kElevatorIntakingUp)) {
      elevator_height_ = 0.29 + 0.3;
      wrist_angle_ = 2.14;
    }


    if (data.IsPressed(kRelease)) {
      top_ = false;
      bottom_ = false;
    }

    new_superstructure_goal->suction.top = top_;
    new_superstructure_goal->suction.bottom = bottom_;

    new_superstructure_goal->elevator.unsafe_goal = elevator_height_;
    new_superstructure_goal->wrist.unsafe_goal = wrist_angle_;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

  void Disc() {
    top_ = true;
    bottom_ = true;
  }
  void Ball() {
    top_ = false;
    bottom_ = true;
  }

 private:
  // Current goals here.
  double elevator_height_ = 0.0;
  double wrist_angle_ = 0.0;

  bool top_ = false;
  bool bottom_ = false;
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
