#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/actions/actions.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/input/driver_station_data.h"
#include "aos/input/joystick_input.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"

#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "y2012/control_loops/accessories/accessories_generated.h"

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
            event_loop->MakeSender<::frc971::control_loops::drivetrain::Goal>(
                "/drivetrain")),
        accessories_goal_sender_(
            event_loop
                ->MakeSender<::y2012::control_loops::accessories::Message>(
                    "/accessories")),
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
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    frc971::control_loops::drivetrain::Goal::Builder goal_builder =
        builder.MakeBuilder<frc971::control_loops::drivetrain::Goal>();
    goal_builder.add_wheel(wheel);
    goal_builder.add_throttle(throttle);
    goal_builder.add_highgear(is_high_gear_);
    goal_builder.add_quickturn(data.IsPressed(kQuickTurn));

    if (!builder.Send(goal_builder.Finish())) {
      AOS_LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    auto builder = accessories_goal_sender_.MakeBuilder();
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> solenoids_offset =
        builder.fbb()->CreateVector<uint8_t>({data.IsPressed(kLongShot),
                                     data.IsPressed(kCloseShot),
                                     data.IsPressed(kFenderShot)});

    flatbuffers::Offset<flatbuffers::Vector<double>> sticks_offset =
        builder.fbb()->CreateVector<double>({data.GetAxis(kAdjustClawGoal),
                                     data.GetAxis(kAdjustClawSeparation)});

    y2012::control_loops::accessories::Message::Builder message_builder =
        builder.MakeBuilder<y2012::control_loops::accessories::Message>();
    message_builder.add_solenoids(solenoids_offset);
    message_builder.add_sticks(sticks_offset);
    if (!builder.Send(message_builder.Finish())) {
      AOS_LOG(WARNING, "sending accessories goal failed\n");
    }
  }

 private:
  ::aos::Sender<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_sender_;
  ::aos::Sender<::y2012::control_loops::accessories::Message>
      accessories_goal_sender_;

  bool is_high_gear_;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2012

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2012::input::joysticks::Reader reader(&event_loop);

  event_loop.Run();

  ::aos::Cleanup();
}
