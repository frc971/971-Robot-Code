#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/atom_code/init.h"
#include "aos/atom_code/input/joystick_input.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/queues/CameraTarget.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::shifters;
using ::frc971::sensors::gyro;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::index_loop;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::angle_adjust;
using ::frc971::control_loops::hangers;
using ::frc971::vision::target_angle;

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

const ButtonLocation kLongShot(3, 5);
const ButtonLocation kMediumShot(3, 3);
const ButtonLocation kShortShot(3, 6);
const ButtonLocation kPitShot1(2, 7), kPitShot2(2, 10);

const ButtonLocation kWristDown(3, 8);

const ButtonLocation kFire(3, 11);
const ButtonLocation kIntake(3, 10);
const ButtonLocation kForceFire(3, 12);
const ButtonLocation kForceIndexUp(3, 9), kForceIndexDown(3, 7);
const ButtonLocation kForceSpitOut(2, 11);

const ButtonLocation kDeployHangers(3, 1);

class Reader : public ::aos::input::JoystickInput {
 public:
  static const bool kWristAlwaysDown = false;

  Reader() {
    shifters.MakeWithBuilder().set(true).Send();
  }

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    static bool is_high_gear = false;
    static double angle_adjust_goal = 0.42;

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
      LOG(DEBUG, "wheel %f throttle %f\n", wheel, throttle);
      const double kThrottleGain = 1.0 / 2.5;
      if (data.IsPressed(kDriveControlLoopEnable1) ||
          data.IsPressed(kDriveControlLoopEnable2)) {
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

      // Whether we should change wrist positions to indicate that the hopper is
      // clear.
      bool hopper_clear = false;

      // Where the wrist should be to pick up a frisbee.
      // TODO(brians): Make these globally accessible and clean up auto.
      static const double kWristPickup = -0.580;
      static const double kWristNearGround = -0.4;
      // Where the wrist gets stored when up.
      // All the way up is 1.5.
      static const double kWristUp = 1.43;
      static const double kWristCleared = kWristUp - 0.2;
      static double wrist_down_position = kWristPickup;
      double wrist_up_position = kWristUp;
      double wrist_pickup_position = data.IsPressed(kIntake) ?
          kWristPickup : kWristNearGround;
      if (index_loop.status.FetchLatest() || index_loop.status.get()) {
        if (index_loop.status->hopper_disc_count >= 4) {
          wrist_down_position = kWristNearGround;
        } else {
          wrist_down_position = wrist_pickup_position;
        }
        hopper_clear = index_loop.status->hopper_clear;
      }

      ::aos::ScopedMessagePtr<control_loops::ShooterLoop::Goal> shooter_goal =
          shooter.goal.MakeMessage();
      shooter_goal->velocity = 0;
      if (data.IsPressed(kPitShot1) && data.IsPressed(kPitShot2)) {
        shooter_goal->velocity = 131;
        if (hopper_clear) wrist_up_position = kWristCleared;
        angle_adjust_goal = 0.70;
      } else if (data.IsPressed(kLongShot)) {
        target_angle.FetchLatest();
        if (target_angle.IsNewerThanMS(500)) {
          shooter_goal->velocity = target_angle->shooter_speed;
          angle_adjust_goal = target_angle->shooter_angle;
          // TODO(brians): do the math right here
          if (!hopper_clear) wrist_up_position = 0.70;
        } else {
          LOG(WARNING, "camera frame too old\n");
          // Pretend like no button is pressed.
        }
      } else if (data.IsPressed(kMediumShot)) {
        // middle wheel on the back line (same as auto)
        shooter_goal->velocity = 395;
        if (!hopper_clear) wrist_up_position = 1.23 - 0.4;
        angle_adjust_goal = 0.520;
      } else if (data.IsPressed(kShortShot)) {
        shooter_goal->velocity = 375;
        if (hopper_clear) wrist_up_position = kWristCleared;
        angle_adjust_goal = 0.671;
      }
      angle_adjust.goal.MakeWithBuilder().goal(angle_adjust_goal).Send();

      wrist.goal.MakeWithBuilder()
          .goal(data.IsPressed(kWristDown) ?
                wrist_down_position :
                wrist_up_position)
          .Send();

      ::aos::ScopedMessagePtr<control_loops::IndexLoop::Goal> index_goal =
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
      const bool spit_out = data.IsPressed(kForceSpitOut);
      index_goal->override_index = index_up || index_down || spit_out;
      index_goal->override_transfer = spit_out;
      if (index_up && index_down) {
        index_goal->index_voltage = 0.0;
      } else if (index_up) {
        index_goal->index_voltage = 12.0;
      } else if (index_down) {
        index_goal->index_voltage = -12.0;
      }
      if (spit_out) {
        index_goal->index_voltage = -12.0;
        index_goal->transfer_voltage = -12.0;
      }

      index_goal.Send();
      shooter_goal.Send();
    }

    static int hanger_cycles = 0;
    if (data.IsPressed(kDeployHangers)) {
      ++hanger_cycles;
      angle_adjust_goal = 0.4;
    } else {
      hanger_cycles = 0;
    }
    hangers.MakeWithBuilder().set(hanger_cycles >= 10).Send();
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
