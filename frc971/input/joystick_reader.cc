#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro_angle.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::sensors::gyro;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace frc971 {
namespace input {
namespace joysticks {

const ButtonLocation kDriveControlLoopEnable1(1, 7),
                     kDriveControlLoopEnable2(1, 11);
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
const ButtonLocation kShiftHigh(2, 1), kShiftLow(2, 3);
const ButtonLocation kQuickTurn(1, 5);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader() {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    static bool is_high_gear = false;

    if (data.GetControlBit(ControlBit::kAutonomous)) {
      if (data.PosEdge(ControlBit::kEnabled)){
        LOG(INFO, "Starting auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(true).Send();
      } else if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "Stopping auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(false).Send();
      }
    } else {  // teleop
      bool is_control_loop_driving = false;
      double left_goal = 0.0;
      double right_goal = 0.0;
      const double wheel = -data.GetAxis(kSteeringWheel);
      const double throttle = -data.GetAxis(kDriveThrottle);
      const double kThrottleGain = 1.0 / 2.5;
      if (false && (data.IsPressed(kDriveControlLoopEnable1) ||
                    data.IsPressed(kDriveControlLoopEnable2))) {
        static double distance = 0.0;
        static double angle = 0.0;
        static double filtered_goal_distance = 0.0;
        if (data.PosEdge(kDriveControlLoopEnable1) ||
            data.PosEdge(kDriveControlLoopEnable2)) {
          if (drivetrain.position.FetchLatest() && gyro.FetchLatest()) {
            distance = (drivetrain.position->left_encoder +
                        drivetrain.position->right_encoder) / 2.0
                - throttle * kThrottleGain / 2.0;
            angle = gyro->angle;
            filtered_goal_distance = distance;
          }
        }
        is_control_loop_driving = true;

        //const double gyro_angle = Gyro.View().angle;
        const double goal_theta = angle - wheel * 0.27;
        const double goal_distance = distance + throttle * kThrottleGain;
        const double robot_width = 22.0 / 100.0 * 2.54;
        const double kMaxVelocity = 0.6;
        if (goal_distance > kMaxVelocity * 0.02 + filtered_goal_distance) {
          filtered_goal_distance += kMaxVelocity * 0.02;
        } else if (goal_distance < -kMaxVelocity * 0.02 +
                   filtered_goal_distance) {
          filtered_goal_distance -= kMaxVelocity * 0.02;
        } else {
          filtered_goal_distance = goal_distance;
        }
        left_goal = filtered_goal_distance - robot_width * goal_theta / 2.0;
        right_goal = filtered_goal_distance + robot_width * goal_theta / 2.0;
        is_high_gear = false;

        LOG(DEBUG, "Left goal %f Right goal %f\n", left_goal, right_goal);
      }
      if (!drivetrain.goal.MakeWithBuilder()
          .steering(wheel)
          .throttle(throttle)
          .highgear(is_high_gear).quickturn(data.IsPressed(kQuickTurn))
          .control_loop_driving(is_control_loop_driving)
          .left_goal(left_goal).right_goal(right_goal).Send()) {
        LOG(WARNING, "sending stick values failed\n");
      }

      if (data.PosEdge(kShiftHigh)) {
        is_high_gear = false;
      }
      if (data.PosEdge(kShiftLow)) {
        is_high_gear = true;
      }

      if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
          .bottom_angle(0)
          .separation_angle(0)
          .intake(false).Send()) {
        LOG(WARNING, "sending claw goal failed\n");
      }
      if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
          .shot_power(0)
          .shot_requested(false)
          .unload_requested(true)
          .Send()) {
        LOG(WARNING, "sending shooter goal failed\n");
      }
    }
  }
};

}  // namespace joysticks
}  // namespace input
}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::input::joysticks::Reader reader;
  reader.Run();
  ::aos::Cleanup();
}
