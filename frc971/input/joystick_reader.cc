#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "aos/linux_code/init.h"
#include "aos/prime/input/joystick_input.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/queues/gyro_angle.q.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/actions/shoot_action.q.h"

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

const ButtonLocation kFire(3, 11);
const ButtonLocation kUnload(2, 11);
const ButtonLocation kReload(2, 6);

const ButtonLocation kRollersOut(3, 12);
const ButtonLocation kRollersIn(3, 10);

const ButtonLocation kTuck(3, 8);
const ButtonLocation kIntakeOpenPosition(3, 9);
const ButtonLocation kIntakePosition(3, 7);

const ButtonLocation kLongShot(3, 5);
const ButtonLocation kMediumShot(3, 3);
const ButtonLocation kShortShot(3, 6);

struct ClawGoal {
  double angle;
  double separation;
};

const ClawGoal kTuckGoal = {-2.273474, -0.749484};
const ClawGoal kIntakeGoal = {-2.273474, 0.0};
const ClawGoal kIntakeOpenGoal = {-2.0, 1.2};

const ClawGoal kLongShotGoal = {-M_PI / 2.0 + 0.43, 0.10};
const ClawGoal kMediumShotGoal = {-0.9, 0.10};
const ClawGoal kShortShotGoal = {-0.04, 0.11};

class Reader : public ::aos::input::JoystickInput {
 public:
  Reader()
      : is_high_gear_(false),
        shot_power_(80.0),
        goal_angle_(0.0),
        separation_angle_(0.0) {}

  virtual void RunIteration(const ::aos::input::driver_station::Data &data) {

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
      // TODO(austin): Static sucks!
      static double distance = 0.0;
      static double angle = 0.0;
      static double filtered_goal_distance = 0.0;
      if (data.PosEdge(kDriveControlLoopEnable1) ||
          data.PosEdge(kDriveControlLoopEnable2)) {
        if (drivetrain.position.FetchLatest() && gyro.FetchLatest()) {
          distance = (drivetrain.position->left_encoder +
                      drivetrain.position->right_encoder) /
                         2.0 -
                     throttle * kThrottleGain / 2.0;
          angle = gyro->angle;
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

  void SetGoal(ClawGoal goal) {
    goal_angle_ = goal.angle;
    separation_angle_ = goal.separation;
  }

  // cycle throught all known actions and if they are running
  // cancel them all.
  // TODO(ben): make this more generic and s.t. we can only
  // cancel if we need those exact resources (ie claw shooter drivetrain)
  void CancelAllActions() {
    frc971::actions::shoot_action.status
        .FetchLatest();
	while (::frc971::actions::shoot_action.status->running) {
      ::frc971::actions::shoot_action.goal.MakeWithBuilder().run(false);
      ::frc971::actions::shoot_action.status.FetchLatest();
    }
  }

  void HandleTeleop(const ::aos::input::driver_station::Data &data) {
    HandleDrivetrain(data);

    if (data.IsPressed(kIntakeOpenPosition)) {
      CancelAllActions();
      SetGoal(kIntakeOpenGoal);
    } else if (data.IsPressed(kIntakePosition)) {
      CancelAllActions();
      SetGoal(kIntakeGoal);
    } else if (data.IsPressed(kTuck)) {
      CancelAllActions();
      SetGoal(kTuckGoal);
    }

    if (data.PosEdge(kLongShot)) {
      CancelAllActions();
      ::frc971::actions::shoot_action.goal.MakeWithBuilder().run(true)
          .shot_power(160.0).shot_angle(kLongShotGoal.angle).Send();
    } else if (data.IsPressed(kMediumShot)) {
      CancelAllActions();
      ::frc971::actions::shoot_action.goal.MakeWithBuilder().run(true)
          .shot_power(100.0).shot_angle(kMediumShotGoal.angle).Send();
    } else if (data.IsPressed(kShortShot)) {
      CancelAllActions();
      ::frc971::actions::shoot_action.goal.MakeWithBuilder().run(true)
          .shot_power(70.0).shot_angle(kShortShotGoal.angle).Send();
    }
  }

 private:
  bool is_high_gear_;
  double shot_power_;
  double goal_angle_;
  double separation_angle_;
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
