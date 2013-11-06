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
 public:
  static const bool kWristAlwaysDown = false;

  Reader() {
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
      }
    } else {  // teleop
      bool is_control_loop_driving = false;
      double left_goal = 0.0;
      double right_goal = 0.0;
      const double wheel = data.GetAxis(kSteeringWheel);
      const double throttle = -data.GetAxis(kDriveThrottle);
      LOG(DEBUG, "wheel %f throttle %f\n", wheel, throttle);
      //const double kThrottleGain = 1.0 / 2.5;
      if (data.IsPressed(kDriveControlLoopEnable1) ||
          data.IsPressed(kDriveControlLoopEnable2)) {
          LOG(INFO, "Control loop driving is currently not supported by this robot.\n");
#if 0
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
#endif
      }
      if (!(drivetrain.goal.MakeWithBuilder()
                .steering(wheel)
                .throttle(throttle)
                .highgear(is_high_gear).quickturn(data.IsPressed(kQuickTurn))
                .control_loop_driving(is_control_loop_driving)
                .left_goal(left_goal).right_goal(right_goal).Send())) {
        LOG(WARNING, "sending stick values failed\n");
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
        velocity = 50;
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
      shooter.goal.MakeWithBuilder().intake(intake).velocity(velocity).push(push).Send();
#if 0
      ::aos::ScopedMessagePtr<frc971::control_loops::ShooterLoop::Goal> shooter_goal =
          shooter.goal.MakeMessage();
      shooter_goal->velocity = 0;
      if (data.IsPressed(kPitShot1) && data.IsPressed(kPitShot2)) {
        shooter_goal->velocity = 131;
      } else if (data.IsPressed(kLongShot)) {
#if 0
        target_angle.FetchLatest();
        if (target_angle.IsNewerThanMS(500)) {
          shooter_goal->velocity = target_angle->shooter_speed;
          angle_adjust_goal = target_angle->shooter_angle;
          // TODO(brians): do the math right here
          wrist_up_position = 0.70;
        } else {
          LOG(WARNING, "camera frame too old\n");
          // pretend like no button is pressed
        }
#endif
        shooter_goal->velocity = 360;
      } else if (data.IsPressed(kMediumShot)) {
#if 0
        shooter_goal->velocity = 375;
        wrist_up_position = 0.70;
        angle_adjust_goal = 0.564;
#endif
        // middle wheel on the back line (same as auto)
        shooter_goal->velocity = 395;
      } else if (data.IsPressed(kShortShot)) {
        shooter_goal->velocity = 375;
      }

      //TODO (daniel) Modify this for hopper and shooter.
      ::aos::ScopedMessagePtr<frc971::control_loops::IndexLoop::Goal> index_goal =
          index_loop.goal.MakeMessage();
      if (data.IsPressed(kFire)) {
        // FIRE
        index_goal->goal_state = 4;
      } else if (shooter_goal->velocity != 0) {
        // get ready to shoot
        index_goal->goal_state = 3;
      } else if (data.IsPressed(kIntake)) {
        // intake
        index_goal->goal_state = 2;
      } else {
        // get ready to intake
        index_goal->goal_state = 1;
      }
      index_goal->force_fire = data.IsPressed(kForceFire);

      const bool index_up = data.IsPressed(kForceIndexUp);
      const bool index_down = data.IsPressed(kForceIndexDown);
      index_goal->override_index = index_up || index_down;
      if (index_up && index_down) {
        index_goal->index_voltage = 0.0;
      } else if (index_up) {
        index_goal->index_voltage = 12.0;
      } else if (index_down) {
        index_goal->index_voltage = -12.0;
      }

      index_goal.Send();
      shooter_goal.Send();
#endif
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
