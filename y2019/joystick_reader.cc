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
const ButtonLocation kElevatorUp(4, 4);
const ButtonLocation kIntakeOut(3, 3);
const ButtonLocation kElevatorDown(0, 0);
const ButtonLocation kElevator1(4, 1);
const ButtonLocation kElevator2(4, 11);
const ButtonLocation kElevator3(4, 9);
const ButtonLocation kElevator4(4, 7);
const ButtonLocation kElevator5(4, 5);

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
const ButtonLocation kDeployStilt(3, 8);
const ButtonLocation kRetractStilt(0, 0);
const ButtonLocation kBackwards(0, 0);

const ButtonLocation kWristDown(3, 1);

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

    // TODO(sabina): max height please?
    if (data.IsPressed(kDeployStilt)) {
      new_superstructure_goal->stilts.unsafe_goal = 0.3;
    } else {
      new_superstructure_goal->stilts.unsafe_goal = 0.01;
    }

    if (data.IsPressed(kIntakeOut)) {
      new_superstructure_goal->intake.unsafe_goal = 0.8;
      new_superstructure_goal->roller_voltage = 5.0;
    } else {
      new_superstructure_goal->intake.unsafe_goal = -1.2;
      new_superstructure_goal->roller_voltage = 0.0;
    }

    if (data.IsPressed(kElevator1)) {
      elevator_height_ = 1.5;
    } else if (data.IsPressed(kElevator2)) {
      elevator_height_ = 1.2;
    } else if (data.IsPressed(kElevator3)) {
      elevator_height_ = 0.8;
    } else if (data.IsPressed(kElevator4)) {
      elevator_height_ = 0.3;
    } else if (data.IsPressed(kElevator5)) {
      elevator_height_ = 0.01;
    }

    if (data.IsPressed(kWristDown)) {
      wrist_angle_ = -M_PI / 3.0;
    } else {
      wrist_angle_ = M_PI / 3.0;
    }

    new_superstructure_goal->elevator.unsafe_goal = elevator_height_;
    new_superstructure_goal->wrist.unsafe_goal = wrist_angle_;

    LOG_STRUCT(DEBUG, "sending goal", *new_superstructure_goal);
    if (!new_superstructure_goal.Send()) {
      LOG(ERROR, "Sending superstructure goal failed.\n");
    }
  }

 private:
  // Current goals here.
  double elevator_height_ = 0.0;
  double wrist_angle_ = 0.0;
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
