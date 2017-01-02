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
#include "aos/common/actions/actions.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"

#include "frc971/queues/gyro.q.h"
#include "y2016_bot4/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain_queue;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace y2016_bot4 {
namespace input {
namespace joysticks {

//#define XBOX

#ifdef XBOX
// Xbox
const JoystickAxis kSteeringWheel(1, 5), kDriveThrottle(1, 2);
const ButtonLocation kQuickTurn(1, 5);
#else
// Steering wheel
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kQuickTurn(1, 5);
#endif

const ButtonLocation kTurn1(1, 7);
const ButtonLocation kTurn2(1, 11);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {}

  void RunIteration(const ::aos::input::driver_station::Data &data) override {
    const bool auto_running = data.GetControlBit(ControlBit::kAutonomous) &&
                              data.GetControlBit(ControlBit::kEnabled);

    if (!auto_running) {
      HandleDrivetrain(data);
    } else {
      drivetrain_queue.goal.MakeWithBuilder()
          .steering(0.0)
          .throttle(0.0)
          .quickturn(false)
          .control_loop_driving(false)
          .left_goal(0.0)
          .right_goal(0.0)
          .left_velocity_goal(0)
          .right_velocity_goal(0)
          .Send();
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
    bool is_control_loop_driving = false;
    const double wheel = -data.GetAxis(kSteeringWheel);
    const double throttle = -data.GetAxis(kDriveThrottle);
    drivetrain_queue.status.FetchLatest();

    if (data.PosEdge(kTurn1) || data.PosEdge(kTurn2)) {
      if (drivetrain_queue.status.get()) {
        left_goal_ = drivetrain_queue.status->estimated_left_position;
        right_goal_ = drivetrain_queue.status->estimated_right_position;
      }
    }
    if (data.IsPressed(kTurn1) || data.IsPressed(kTurn2)) {
      is_control_loop_driving = true;
    }
    if (!drivetrain_queue.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal_ - wheel * 0.5 + throttle * 0.3)
             .right_goal(right_goal_ + wheel * 0.5 + throttle * 0.3)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
             .Send()) {
      LOG(WARNING, "sending stick values failed\n");
    }
  }

 private:
  double left_goal_ = 0.0;
  double right_goal_ = 0.0;
};

}  // namespace joysticks
}  // namespace input
}  // namespace y2016_bot4

int main() {
  ::aos::Init(-1);
  ::y2016_bot4::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
