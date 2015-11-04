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

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/drivetrain/drivetrain_constants.h"
#include "bot3/control_loops/rollers/rollers.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/queues/other_sensors.q.h"

using ::bot3::control_loops::drivetrain;
using ::frc971::sensors::gyro_reading;

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::ControlBit;

namespace bot3 {
namespace input {
namespace joysticks {

const ButtonLocation kDriveControlLoopEnable1(1, 7),
                     kDriveControlLoopEnable2(1, 11);
const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(4, 2);
const ButtonLocation kShiftHigh(4, 1), kShiftLow(4, 3);
const ButtonLocation kQuickTurn(1, 5);

const ButtonLocation kFrontRollersIn(3, 5);
const ButtonLocation kBackRollersIn(3, 3);
const ButtonLocation kFrontRollersOut(3, 12);
const ButtonLocation kBackRollersOut(3, 8);
const ButtonLocation kHumanPlayer(3, 11);

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {
    if (data.GetControlBit(ControlBit::kAutonomous)) {
      if (data.PosEdge(ControlBit::kEnabled)){
        LOG(INFO, "Starting auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(true)
            .Send();
      } else if (data.NegEdge(ControlBit::kEnabled)) {
        LOG(INFO, "Stopping auto mode\n");
        ::frc971::autonomous::autonomous.MakeWithBuilder()
            .run_auto(false)
            .Send();
      } else if (!data.GetControlBit(ControlBit::kEnabled)) {
        auto goal = drivetrain.goal.MakeMessage();
        goal->Zero();
        goal->control_loop_driving = false;
        goal->left_goal = goal->right_goal = 0;
        goal->left_velocity_goal = goal->right_velocity_goal = 0;
        if (!goal.Send()) {
          LOG(WARNING, "sending 0 drivetrain goal failed\n");
        }
      }
    } else {
      HandleTeleop(data);
    }
  }

  void HandleDrivetrain(const ::aos::input::driver_station::Data &data) {
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
        if (drivetrain.position.FetchLatest() && gyro_reading.FetchLatest()) {
          distance = (drivetrain.position->left_encoder +
                      drivetrain.position->right_encoder) /
                         2.0 -
                     throttle * kThrottleGain / 2.0;
          angle = gyro_reading->angle;
          filtered_goal_distance = distance;
        }
      }
      is_control_loop_driving = true;

      // const double gyro_angle = Gyro.View().angle;
      const double goal_theta = angle - wheel * 0.27;
      const double goal_distance = distance + throttle * kThrottleGain;
      const double robot_width = 22.0 / 100.0 * 2.54;
      const double kMaxVelocity = 0.6;
      if (goal_distance > kMaxVelocity * 0.02 + filtered_goal_distance) {
        filtered_goal_distance += kMaxVelocity * 0.02;
      } else if (goal_distance <
                 -kMaxVelocity * 0.02 + filtered_goal_distance) {
        filtered_goal_distance -= kMaxVelocity * 0.02;
      } else {
        filtered_goal_distance = goal_distance;
      }
      left_goal = filtered_goal_distance - robot_width * goal_theta / 2.0;
      right_goal = filtered_goal_distance + robot_width * goal_theta / 2.0;
      is_high_gear_ = false;

      LOG(DEBUG, "Left goal %f Right goal %f\n", left_goal, right_goal);
    }
    if (!drivetrain.goal.MakeWithBuilder()
             .steering(wheel)
             .throttle(throttle)
             .highgear(is_high_gear_)
             .quickturn(data.IsPressed(kQuickTurn))
             .control_loop_driving(is_control_loop_driving)
             .left_goal(left_goal)
             .right_goal(right_goal)
             .left_velocity_goal(0)
             .right_velocity_goal(0)
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
    HandleDrivetrain(data);

    // Rollers.
    auto rollers_goal = control_loops::rollers.goal.MakeMessage();
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
  bool is_high_gear_;
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
