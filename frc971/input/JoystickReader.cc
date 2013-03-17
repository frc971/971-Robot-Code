#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/aos_core.h"
#include "aos/atom_code/input/FRCComm.h"
#include "aos/atom_code/input/JoystickInput.h"

#include "frc971/input/AutoMode.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/GyroAngle.q.h"
#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::shifters;
using ::frc971::sensors::gyro;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::index_loop;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::angle_adjust;

namespace frc971 {

class JoystickReader : public aos::JoystickInput {
 public:
  static const bool kWristAlwaysDown = false;

  JoystickReader() : aos::JoystickInput() {
    shifters.MakeWithBuilder().set(true).Send();
  }

  virtual void RunIteration() {
    static bool is_high_gear = false;

    if (Pressed(0, AUTONOMOUS)) {
      if (PosEdge(0, ENABLED)){
        LOG(INFO, "Starting auto mode\n");
        AutoMode.Start();
      }
      if (NegEdge(0, ENABLED)) {
        LOG(INFO, "Stopping auto mode\n");
        AutoMode.Stop();
      }
    } else {  // teleop
      bool is_control_loop_driving = false;
      double left_goal = 0.0;
      double right_goal = 0.0;
      const double wheel = control_data_.stick0Axis1 / 127.0;
      const double throttle = -control_data_.stick1Axis2 / 127.0;
      LOG(DEBUG, "wheel %f throttle %f\n", wheel, throttle);
      const double kThrottleGain = 1.0 / 2.5;
      if (Pressed(0, 7) || Pressed(0, 11)) {
        static double distance = 0.0;
        static double angle = 0.0;
        static double filtered_goal_distance = 0.0;
        if (PosEdge(0, 7) || PosEdge(0, 11)) {
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
        } else if (goal_distance < -kMaxVelocity * 0.02 + filtered_goal_distance) {
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
                .highgear(is_high_gear).quickturn(Pressed(0, 5))
                .control_loop_driving(is_control_loop_driving)
                .left_goal(left_goal).right_goal(right_goal).Send())) {
        LOG(WARNING, "sending stick values failed\n");
      }

      if (PosEdge(1, 1)) {
        is_high_gear = false;
      }
      if (PosEdge(1, 3)) {
        is_high_gear = true;
      }

      // Where the wrist should be to pick up a frisbee.
      static const double kWristPickup = -0.633;
      // Where the wrist gets stored when up.
      // All the way up is 1.5.
      static const double kWristUp = 1.43;
      static double wrist_down_position = kWristPickup;
      index_loop.status.FetchLatest();
      if (index_loop.status.get()) {
        if (index_loop.status->hopper_disc_count >= 4) {
          wrist_down_position = -0.4;
        } else {
          wrist_down_position = kWristPickup;
        }
      }
      wrist.goal.MakeWithBuilder()
          .goal(Pressed(2, 8) ? wrist_down_position : kWristUp).Send();

      ::aos::ScopedMessagePtr<control_loops::ShooterLoop::Goal> shooter_goal =
          shooter.goal.MakeMessage();
      shooter_goal->velocity = 0;
      static double angle_adjust_goal = 0.42;
      if (Pressed(2, 5)) {
        // short shot
        shooter_goal->velocity = 200;
        angle_adjust_goal = 0.42;
      } else if (Pressed(2, 3)) {
        // medium shot
        shooter_goal->velocity = 220;
        angle_adjust_goal = 0.45;
      } else if (Pressed(2, 6)) {
        // long shot
        shooter_goal->velocity = 240;
        angle_adjust_goal = 0.55;
      }
      angle_adjust.goal.MakeWithBuilder().goal(angle_adjust_goal).Send();

      ::aos::ScopedMessagePtr<control_loops::IndexLoop::Goal> index_goal =
          index_loop.goal.MakeMessage();
      // TODO(brians): replace these with the enum values
      if (Pressed(2, 11)) {
        // FIRE
        index_goal->goal_state = 4;
      } else if (shooter_goal->velocity != 0) {
        // get ready to shoot
        index_goal->goal_state = 3;
      } else if (Pressed(2, 10)) {
        // intake
        index_goal->goal_state = 2;
      } else {
        // get ready to intake
        index_goal->goal_state = 1;
      }

      index_goal.Send();
      shooter_goal.Send();
    }
  }
};

}  // namespace frc971

AOS_RUN(frc971::JoystickReader)
