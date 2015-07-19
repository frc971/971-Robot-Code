#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/actions/actions.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro.q.h"
#include "bot3/autonomous/auto.q.h"

using ::bot3::control_loops::drivetrain_queue;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::POVLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace bot3 {
namespace input {
namespace joysticks {

const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

// TODO(comran): Figure out the actions we need.

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() : was_running_(false) {}

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
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    if (!data.GetControlBit(ControlBit::kEnabled)) {
      action_queue_.CancelAllActions();
      LOG(DEBUG, "Canceling\n");
    }

    // TODO(comran): Run stuff when buttons are pushed.

    // TODO(comran): If it cannot get a status from the superstructure...
    /*
    if (!superstructure_queue.status.get()) {
      LOG(ERROR, "Got no superstructure status packet.\n");
    }
    */

    // TODO(comran): If everything looks good, start teleop.
    /*if (superstructure_queue.status.get() && superstructure_queue.status->zeroed) {
      if (waiting_for_zero_) {
        LOG(INFO, "Zeroed! Starting teleop mode.\n");
        waiting_for_zero_ = false;

        // Set the initial goals to where we are now.
        superstructure_goal_ = 0.0;
      }
    } else {
      waiting_for_zero_ = true;
    }*/

    if (!waiting_for_zero_) {
      if (!action_queue_.Running()) {
        // TODO(comran): Send some goals.
      }
    }

    if (action_queue_.Running()) {
      // If we are running an action, update our goals to the current goals.
      // TODO(comran): Do this for each superstructure queue.
    }
    action_queue_.Tick();
    was_running_ = action_queue_.Running();
  }

 private:
  void StartAuto() {
    LOG(INFO, "Starting auto mode\n");
    ::bot3::autonomous::autonomous.MakeWithBuilder().run_auto(true).Send();
  }

  void StopAuto() {
    LOG(INFO, "Stopping auto mode\n");
    ::bot3::autonomous::autonomous.MakeWithBuilder().run_auto(false).Send();
  }

  bool was_running_;

  // If we're waiting for the subsystems to zero.
  bool waiting_for_zero_ = true;

  bool auto_running_ = false;

  ::aos::common::actions::ActionQueue action_queue_;

  ::aos::util::SimpleLogInterval no_drivetrain_status_ =
      ::aos::util::SimpleLogInterval(::aos::time::Time::InSeconds(0.2), WARNING,
                                     "no drivetrain status");
};

}  // namespace joysticks
}  // namespace input
}  // namespace bot3

int main() {
  ::aos::Init();
  ::bot3::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
