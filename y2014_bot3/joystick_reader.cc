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

#include "frc971/queues/gyro.q.h"
#include "y2014_bot3/autonomous/auto.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain.q.h"
#include "y2014_bot3/control_loops/rollers/rollers.q.h"

using ::y2014_bot3::control_loops::drivetrain_queue;
using ::y2014_bot3::control_loops::rollers_queue;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace y2014_bot3 {
namespace input {
namespace joysticks {

// Joystick & button addresses.
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kFrontRollersIn(3, 8);
const ButtonLocation kBackRollersIn(3, 7);
const ButtonLocation kFrontRollersOut(3, 6);
const ButtonLocation kBackRollersOut(4, 12);
const ButtonLocation kHumanPlayer(4, 11);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : is_high_gear_(false) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
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

    if (!data.GetControlBit(ControlBit::kAutonomous)) {
      HandleDrivetrain(data);
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);

    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(false)
             .highgear(is_high_gear_)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }

    if (data.PosEdge(kShiftHigh)) {
      is_high_gear_ = false;
    }

    if (data.PosEdge(kShiftLow)) {
      is_high_gear_ = true;
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    // Rollers.
    auto rollers_goal = control_loops::rollers_queue.goal.MakeMessage();
    rollers_goal->Zero();
    if (data.IsPressed(kFrontRollersIn)) {
      rollers_goal->intake = 1;
    } else if (data.IsPressed(kFrontRollersOut)) {
      rollers_goal->low_spit = 1;
    } else if (data.IsPressed(kBackRollersIn)) {
      rollers_goal->intake = -1;
    } else if (data.IsPressed(kBackRollersOut)) {
      rollers_goal->low_spit = -1;
    } else if (data.IsPressed(kHumanPlayer)) {
      rollers_goal->human_player = true;
    }
    if (!rollers_goal.Send()) {
      LOG(WARNING, "Sending rollers values failed.\n");
    }
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode.\n");
    ::y2014_bot3::autonomous::autonomous.MakeWithBuilder().run_auto(true).Send();
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    ::y2014_bot3::autonomous::autonomous.MakeWithBuilder().run_auto(false).Send();
  }

  bool auto_running_ = false;

  bool is_high_gear_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2014_bot3

int main() {
  ::aos::Init(-1);
  ::y2014_bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
