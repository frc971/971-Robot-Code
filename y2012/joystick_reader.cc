#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/actions/actions.h"
#include "aos/events/shm-event-loop.h"
#include "aos/init.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/joystick_input.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2012/control_loops/accessories/accessories.q.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

#define OLD_DS 0

namespace y2012 {
namespace input {
namespace joysticks {

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 3), kShiftLow(2, 1);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kCatch(3, 10);

#if OLD_DS
const ButtonLocation kFire(3, 11);
const ButtonLocation kUnload(1, 4);
const ButtonLocation kReload(1, 2);

const ButtonLocation kRollersOut(3, 12);
const ButtonLocation kRollersIn(3, 7);

const ButtonLocation kTuck(3, 9);
const ButtonLocation kIntakePosition(3, 8);
const ButtonLocation kIntakeOpenPosition(3, 10);
const ButtonLocation kVerticalTuck(3, 1);
const JoystickAxis kFlipRobot(3, 3);

const ButtonLocation kLongShot(3, 5);
const ButtonLocation kCloseShot(3, 2);
const ButtonLocation kFenderShot(3, 3);
const ButtonLocation kTrussShot(2, 11);
const ButtonLocation kHumanPlayerShot(3, 2);
#else
const ButtonLocation kFire(3, 9);
const ButtonLocation kUnload(1, 4);
const ButtonLocation kReload(1, 2);

const ButtonLocation kRollersOut(3, 8);
const ButtonLocation kRollersIn(3, 3);

const ButtonLocation kTuck(3, 4);
const ButtonLocation kIntakePosition(3, 5);
const ButtonLocation kIntakeOpenPosition(3, 11);
const ButtonLocation kVerticalTuck(2, 6);
const JoystickAxis kFlipRobot(3, 3);

const ButtonLocation kLongShot(3, 7);
const ButtonLocation kCloseShot(3, 6);
const ButtonLocation kFenderShot(3, 2);
const ButtonLocation kTrussShot(2, 11);
const ButtonLocation kHumanPlayerShot(3, 1);
#endif

const ButtonLocation kUserLeft(2, 7);
const ButtonLocation kUserRight(2, 10);

const JoystickAxis kAdjustClawGoal(3, 2);
const JoystickAxis kAdjustClawSeparation(3, 1);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop)
      : ::aos::input::JoystickInput(event_loop),
        drivetrain_goal_sender_(
            event_loop
                ->MakeSender<::frc971::control_loops::DrivetrainQueue::Goal>(
                    ".frc971.control_loops.drivetrain_queue.goal")),
        accessories_goal_sender_(
            event_loop
                ->MakeSender<::y2012::control_loops::AccessoriesQueue::Message>(
                    ".y2012.control_loops.accessories_queue.goal")),
        is_high_gear_(false) {}

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);
    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = false;
    }
    if (data.PosEdge(kShiftHigh)) {
      is_high_gear_ = true;
    }
    auto drivetrain_message = drivetrain_goal_sender_.MakeMessage();
    drivetrain_message->wheel = wheel;
    drivetrain_message->throttle = throttle;
    drivetrain_message->highgear = is_high_gear_;
    drivetrain_message->quickturn = data.IsPressed(kQuickTurn);

    if (!drivetrain_message.Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    auto accessories_message = accessories_goal_sender_.MakeMessage();
    accessories_message->solenoids[0] = data.IsPressed(kLongShot);
    accessories_message->solenoids[1] = data.IsPressed(kCloseShot);
    accessories_message->solenoids[2] = data.IsPressed(kFenderShot);
    accessories_message->sticks[0] = data.GetAxis(kAdjustClawGoal);
    accessories_message->sticks[1] = data.GetAxis(kAdjustClawSeparation);
    if (!accessories_message.Send()) {
      LOG(WARNING, "sending accessories goal failed\n");
    }
  }

 private:
  ::aos::Sender<::frc971::control_loops::DrivetrainQueue::Goal>
      drivetrain_goal_sender_;
  ::aos::Sender<::y2012::control_loops::AccessoriesQueue::Message>
      accessories_goal_sender_;

  bool is_high_gear_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2012

int main() {
  ::aos::Init(-1);
  ::aos::ShmEventLoop event_loop;
  ::y2012::input::joysticks::Reader reader(&event_loop);
  reader.Run();
  ::aos::Cleanup();
}
