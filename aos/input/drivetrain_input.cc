#include "aos/input/drivetrain_input.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <cmath>

#include "aos/common/commonmath.h"
#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace aos {
namespace input {

void DrivetrainInputReader::HandleDrivetrain(
    const ::aos::input::driver_station::Data &data) {
  bool is_control_loop_driving = false;

  const auto wheel_and_throttle = GetWheelAndThrottle(data);
  const double wheel = wheel_and_throttle.wheel;
  const double throttle = wheel_and_throttle.throttle;

  drivetrain_queue.status.FetchLatest();
  if (drivetrain_queue.status.get()) {
    robot_velocity_ = drivetrain_queue.status->robot_speed;
  }

  if (data.PosEdge(kTurn1) || data.PosEdge(kTurn2)) {
    if (drivetrain_queue.status.get()) {
      left_goal_ = drivetrain_queue.status->estimated_left_position;
      right_goal_ = drivetrain_queue.status->estimated_right_position;
    }
  }
  const double current_left_goal =
      left_goal_ - wheel * wheel_multiplier_ + throttle * 0.3;
  const double current_right_goal =
      right_goal_ + wheel * wheel_multiplier_ + throttle * 0.3;
  if (data.IsPressed(kTurn1) || data.IsPressed(kTurn2)) {
    is_control_loop_driving = true;
  }
  auto new_drivetrain_goal = drivetrain_queue.goal.MakeMessage();
  new_drivetrain_goal->steering = wheel;
  new_drivetrain_goal->throttle = throttle;
  new_drivetrain_goal->quickturn = data.IsPressed(kQuickTurn);
  new_drivetrain_goal->control_loop_driving = is_control_loop_driving;
  new_drivetrain_goal->left_goal = current_left_goal;
  new_drivetrain_goal->right_goal = current_right_goal;
  new_drivetrain_goal->left_velocity_goal = 0;
  new_drivetrain_goal->right_velocity_goal = 0;

  new_drivetrain_goal->linear.max_velocity = 3.0;
  new_drivetrain_goal->linear.max_acceleration = 20.0;

  if (!new_drivetrain_goal.Send()) {
    LOG(WARNING, "sending stick values failed\n");
  }
}

DrivetrainInputReader::WheelAndThrottle
SteeringWheelDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  const double wheel = -data.GetAxis(kSteeringWheel);
  const double throttle = -data.GetAxis(kDriveThrottle);
  return DrivetrainInputReader::WheelAndThrottle{wheel, throttle};
}

DrivetrainInputReader::WheelAndThrottle
PistolDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  const double unscaled_wheel = data.GetAxis(kSteeringWheel);
  double wheel;
  if (unscaled_wheel < 0.0) {
    wheel = unscaled_wheel / 0.484375;
  } else {
    wheel = unscaled_wheel / 0.385827;
  }

  const double unscaled_throttle = -data.GetAxis(kDriveThrottle);
  double unmodified_throttle;
  if (unscaled_throttle < 0.0) {
    unmodified_throttle = unscaled_throttle / 0.086614;
  } else {
    unmodified_throttle = unscaled_throttle / 0.265625;
  }
  unmodified_throttle = aos::Deadband(unmodified_throttle, 0.1, 1.0);

  // Apply a sin function that's scaled to make it feel better.
  constexpr double throttle_range = M_PI_2 * 0.5;

  double throttle = ::std::sin(throttle_range * unmodified_throttle) /
                    ::std::sin(throttle_range);
  throttle = ::std::sin(throttle_range * throttle) / ::std::sin(throttle_range);
  throttle = 2.0 * unmodified_throttle - throttle;
  return DrivetrainInputReader::WheelAndThrottle{wheel, throttle};
}

DrivetrainInputReader::WheelAndThrottle
XboxDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  // xbox
  constexpr double kWheelDeadband = 0.05;
  constexpr double kThrottleDeadband = 0.05;
  const double wheel =
      aos::Deadband(-data.GetAxis(kSteeringWheel), kWheelDeadband, 1.0);

  const double unmodified_throttle =
      aos::Deadband(-data.GetAxis(kDriveThrottle), kThrottleDeadband, 1.0);

  // Apply a sin function that's scaled to make it feel better.
  constexpr double throttle_range = M_PI_2 * 0.9;

  double throttle = ::std::sin(throttle_range * unmodified_throttle) /
                    ::std::sin(throttle_range);
  throttle = ::std::sin(throttle_range * throttle) / ::std::sin(throttle_range);
  throttle = 2.0 * unmodified_throttle - throttle;
  return DrivetrainInputReader::WheelAndThrottle{wheel, throttle};
}

std::unique_ptr<SteeringWheelDrivetrainInputReader>
SteeringWheelDrivetrainInputReader::Make() {
  const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
  const ButtonLocation kQuickTurn(1, 5);
  const ButtonLocation kTurn1(1, 7);
  const ButtonLocation kTurn2(1, 11);
  std::unique_ptr<SteeringWheelDrivetrainInputReader> result(
      new SteeringWheelDrivetrainInputReader(kSteeringWheel, kDriveThrottle,
                                             kQuickTurn, kTurn1, kTurn2));
  return result;
}

std::unique_ptr<PistolDrivetrainInputReader>
PistolDrivetrainInputReader::Make() {
  // Pistol Grip controller
  const JoystickAxis kSteeringWheel(1, 2), kDriveThrottle(1, 1);
  const ButtonLocation kQuickTurn(1, 7);
  const ButtonLocation kTurn1(1, 8);

  // Nop
  const ButtonLocation kTurn2(1, 9);
  std::unique_ptr<PistolDrivetrainInputReader> result(
      new PistolDrivetrainInputReader(kSteeringWheel, kDriveThrottle,
                                      kQuickTurn, kTurn1, kTurn2));
  result->set_wheel_multiplier(0.2);
  return result;
}

std::unique_ptr<XboxDrivetrainInputReader> XboxDrivetrainInputReader::Make() {
  // xbox
  const JoystickAxis kSteeringWheel(1, 5), kDriveThrottle(1, 2);
  const ButtonLocation kQuickTurn(1, 5);

  // Nop
  const ButtonLocation kTurn1(1, 1);
  const ButtonLocation kTurn2(1, 2);

  std::unique_ptr<XboxDrivetrainInputReader> result(
      new XboxDrivetrainInputReader(kSteeringWheel, kDriveThrottle, kQuickTurn,
                                    kTurn1, kTurn2));
  return result;
}
::std::unique_ptr<DrivetrainInputReader> DrivetrainInputReader::Make(
    InputType type) {
  std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader;

  using InputType = DrivetrainInputReader::InputType;
  switch (type) {
    case InputType::kSteeringWheel:
      drivetrain_input_reader = SteeringWheelDrivetrainInputReader::Make();
      break;
    case InputType::kPistol:
      drivetrain_input_reader = PistolDrivetrainInputReader::Make();
      break;
    case InputType::kXbox:
      drivetrain_input_reader = XboxDrivetrainInputReader::Make();
      break;
  }
  return drivetrain_input_reader;
}

}  // namespace input
}  // namespace aos
