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

      // frisbee pickup is -0.634
      wrist.goal.MakeWithBuilder().goal(-0.634).Send();

      index_loop.goal.MakeWithBuilder()
          .goal_state(Pressed(1, 4) ? 2 :
                      Pressed(1, 5) ? 3 :
                      Pressed(1, 10) ? 4 : 1).Send();

      angle_adjust.goal.MakeWithBuilder()
          .goal(Pressed(3, 1) ? 0.6 : 0.35).Send();

      shooter.goal.MakeWithBuilder()
          .velocity(Pressed(2, 9) ? 325.0 : 0.0).Send();
    }
  }
};

}  // namespace frc971

AOS_RUN(frc971::JoystickReader)
