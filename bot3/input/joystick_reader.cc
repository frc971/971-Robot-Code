#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/atom_code/init.h"
#include "aos/atom_code/input/joystick_input.h"
#include "aos/common/logging/logging.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"
#include "bot3/autonomous/auto.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"
#include "frc971/queues/CameraTarget.q.h"

using ::bot3::control_loops::drivetrain;
using ::bot3::control_loops::shooter;
using ::frc971::control_loops::shifters;
using ::frc971::sensors::gyro;
// using ::frc971::vision::target_angle;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace bot3 {
namespace input {
namespace joysticks {

const ButtonLocation kDriveControlLoopEnable1(1, 7),
                     kDriveControlLoopEnable2(1, 11);
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kPush(3, 9);

const ButtonLocation kFire(3, 3);
const ButtonLocation kIntake(3, 4);

class Reader : public ::aos::input::JoystickInput {
  bool last_push_;
  bool shooting_;
 public:
  static const bool kWristAlwaysDown = false;

  Reader() : shooting_(false) {
    printf("\nRunning Bot3 JoystickReader!\n");
    shifters.MakeWithBuilder().set(true).Send();
  }

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    static bool is_high_gear = false;

    if (data.GetControlBit(ControlBit::kAutonomous)) {
      if (data.PosEdge(ControlBit::kEnabled)){
        LOG(INFO, "Starting auto mode\n");
        ::bot3::autonomous::autonomous.MakeWithBuilder()
            .run_auto(true).Send();
      } else if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "Stopping auto mode\n");
        ::bot3::autonomous::autonomous.MakeWithBuilder()
            .run_auto(false).Send();
      } else {
        LOG(DEBUG, "Running auto\n");
      }
    } else {  // teleop
      bool is_control_loop_driving = false;
      double left_goal = 0.0;
      double right_goal = 0.0;
      const double wheel = data.GetAxis(kSteeringWheel);
      const double throttle = -data.GetAxis(kDriveThrottle);
      const bool quickturn = data.IsPressed(kQuickTurn);
      LOG(DEBUG, "wheel %f throttle %f quickturn %d\n", wheel, throttle, quickturn);
      //const double kThrottleGain = 1.0 / 2.5;
      if (!shooting_) {
        if (!drivetrain.goal.MakeWithBuilder()
                  .steering(wheel)
                  .throttle(throttle)
                  .highgear(is_high_gear).quickturn(quickturn)
                  .control_loop_driving(is_control_loop_driving)
                  .left_goal(left_goal).right_goal(right_goal).Send()) {
          LOG(WARNING, "sending stick values failed\n");
        }
      }

      if (data.PosEdge(kShiftHigh)) {
        is_high_gear = false;
      }
      if (data.PosEdge(kShiftLow)) {
        is_high_gear = true;
      }

      // 3, 4, 9, 9 fires, 3 pickups

      shooter.status.FetchLatest();
      bool push = false;
      double velocity = 0.0;
      double intake = 0.0;
      if (data.IsPressed(kPush) && shooter.status->ready) {
        push = true;
      }
      if (data.IsPressed(kFire)) {
        velocity = 500;
      }
      else if (data.IsPressed(ButtonLocation(3, 1))) {
        velocity = 100;
      }
      else if (data.IsPressed(ButtonLocation(3, 2))) {
        velocity = 250;
      }
      else if (data.IsPressed(ButtonLocation(3, 5))) {
        velocity = 300;
      }
      else if (data.IsPressed(ButtonLocation(3, 7))) {
        velocity = 350;
      }
      else if (data.IsPressed(ButtonLocation(3, 8))) {
        velocity = 400;
      }
      else if (data.IsPressed(ButtonLocation(3, 10))) {
        velocity = 450;
      }
      if (data.IsPressed(kIntake)) {
        intake = 0.8;
      }
      if (abs(throttle) < 0.2 && !quickturn) {
        shooting_ = true;
        shooter.goal.MakeWithBuilder().intake(intake).velocity(velocity).push(push).Send();
      } else {
        shooting_ = false;
      }
      if (!velocity) {
        shooting_ = false;
      }
   }
  }
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
