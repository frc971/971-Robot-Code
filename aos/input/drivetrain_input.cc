#include "aos/input/drivetrain_input.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <cmath>

#include "aos/commonmath.h"
#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"

using ::aos::input::driver_station::ButtonLocation;
using ::aos::input::driver_station::ControlBit;
using ::aos::input::driver_station::JoystickAxis;
using ::aos::input::driver_station::POVLocation;

namespace drivetrain = frc971::control_loops::drivetrain;

namespace aos {
namespace input {

const ButtonLocation kShiftHigh(2, 3), kShiftHigh2(2, 2), kShiftLow(2, 1);

void DrivetrainInputReader::HandleDrivetrain(
    const ::aos::input::driver_station::Data &data) {
  const auto wheel_and_throttle = GetWheelAndThrottle(data);
  const double wheel = wheel_and_throttle.wheel;
  const double wheel_velocity = wheel_and_throttle.wheel_velocity;
  const double wheel_torque = wheel_and_throttle.wheel_torque;
  const double throttle = wheel_and_throttle.throttle;
  const double throttle_velocity = wheel_and_throttle.throttle_velocity;
  const double throttle_torque = wheel_and_throttle.throttle_torque;
  const bool high_gear = wheel_and_throttle.high_gear;

  drivetrain_status_fetcher_.Fetch();
  if (drivetrain_status_fetcher_.get()) {
    robot_velocity_ = drivetrain_status_fetcher_->robot_speed();
  }

  // If we have a vision align function, and it is in control, don't run the
  // normal driving code.
  if (vision_align_fn_) {
    if (vision_align_fn_(data)) {
      return;
    }
  }

  bool is_control_loop_driving = false;
  bool is_line_following = false;

  if (data.IsPressed(turn1_)) {
    switch (turn1_use_) {
      case TurnButtonUse::kControlLoopDriving:
        is_control_loop_driving = true;
        break;
      case TurnButtonUse::kLineFollow:
        is_line_following = true;
        break;
    }
  }

  if (data.IsPressed(turn2_)) {
    switch (turn2_use_) {
      case TurnButtonUse::kControlLoopDriving:
        is_control_loop_driving = true;
        break;
      case TurnButtonUse::kLineFollow:
        is_line_following = true;
        break;
    }
  }

  if (drivetrain_status_fetcher_.get()) {
    if (is_control_loop_driving && !last_is_control_loop_driving_) {
      left_goal_ = drivetrain_status_fetcher_->estimated_left_position() +
                   wheel * wheel_multiplier_;
      right_goal_ = drivetrain_status_fetcher_->estimated_right_position() -
                    wheel * wheel_multiplier_;
    }
  }

  const double current_left_goal =
      left_goal_ - wheel * wheel_multiplier_ + throttle * 0.3;
  const double current_right_goal =
      right_goal_ + wheel * wheel_multiplier_ + throttle * 0.3;
  auto builder = drivetrain_goal_sender_.MakeBuilder();

  frc971::ProfileParameters::Builder linear_builder =
      builder.MakeBuilder<frc971::ProfileParameters>();

  linear_builder.add_max_velocity(3.0);
  linear_builder.add_max_acceleration(20.0);

  flatbuffers::Offset<frc971::ProfileParameters> linear_offset =
      linear_builder.Finish();

  auto goal_builder = builder.MakeBuilder<drivetrain::Goal>();
  goal_builder.add_wheel(wheel);
  goal_builder.add_wheel_velocity(wheel_velocity);
  goal_builder.add_wheel_torque(wheel_torque);
  goal_builder.add_throttle(throttle);
  goal_builder.add_throttle_velocity(throttle_velocity);
  goal_builder.add_throttle_torque(throttle_torque);
  goal_builder.add_highgear(high_gear);
  goal_builder.add_quickturn(data.IsPressed(quick_turn_));
  goal_builder.add_controller_type(
      is_line_following
          ? drivetrain::ControllerType::LINE_FOLLOWER
          : (is_control_loop_driving ? drivetrain::ControllerType::MOTION_PROFILE
                                     : drivetrain::ControllerType::POLYDRIVE));
  goal_builder.add_left_goal(current_left_goal);
  goal_builder.add_right_goal(current_right_goal);
  goal_builder.add_linear(linear_offset);

  if (!builder.Send(goal_builder.Finish())) {
    AOS_LOG(WARNING, "sending stick values failed\n");
  }

  last_is_control_loop_driving_ = is_control_loop_driving;
}

DrivetrainInputReader::WheelAndThrottle
SteeringWheelDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  const double wheel = -data.GetAxis(wheel_);
  const double throttle = -data.GetAxis(throttle_);

  if (!data.GetControlBit(ControlBit::kEnabled)) {
    high_gear_ = default_high_gear_;
  }

  if (data.PosEdge(kShiftLow)) {
    high_gear_ = false;
  }

  if (data.PosEdge(kShiftHigh) || data.PosEdge(kShiftHigh2)) {
    high_gear_ = true;
  }

  return DrivetrainInputReader::WheelAndThrottle{
      wheel, 0.0, 0.0, throttle, 0.0, 0.0, high_gear_};
}

double UnwrappedAxis(const ::aos::input::driver_station::Data &data,
                     const JoystickAxis &high_bits,
                     const JoystickAxis &low_bits) {
  const float high_bits_data = data.GetAxis(high_bits);
  const float low_bits_data = data.GetAxis(low_bits);
  const int16_t high_bits_data_int =
      (high_bits_data < 0.0f ? high_bits_data * 128.0f
                             : high_bits_data * 127.0f);
  const int16_t low_bits_data_int =
      (low_bits_data < 0.0f ? low_bits_data * 128.0f : low_bits_data * 127.0f);

  const uint16_t high_bits_data_uint =
      ((static_cast<uint16_t>(high_bits_data_int) & 0xff) + 0x80) & 0xff;
  const uint16_t low_bits_data_uint =
      ((static_cast<uint16_t>(low_bits_data_int) & 0xff) + 0x80) & 0xff;

  const uint16_t data_uint = (high_bits_data_uint << 8) | low_bits_data_uint;

  const int32_t data_int = static_cast<int32_t>(data_uint) - 0x8000;

  if (data_int < 0) {
    return static_cast<double>(data_int) / static_cast<double>(0x8000);
  } else {
    return static_cast<double>(data_int) / static_cast<double>(0x7fff);
  }
}

DrivetrainInputReader::WheelAndThrottle
PistolDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  const double wheel =
      -UnwrappedAxis(data, wheel_, wheel_low_);
  const double wheel_velocity =
      -UnwrappedAxis(data, wheel_velocity_high_, wheel_velocity_low_) * 50.0;
  const double wheel_torque =
      -UnwrappedAxis(data, wheel_torque_high_, wheel_torque_low_) / 2.0;

  double throttle =
      UnwrappedAxis(data, throttle_, throttle_low_);
  const double throttle_velocity =
      UnwrappedAxis(data, throttle_velocity_high_, throttle_velocity_low_) * 50.0;
  const double throttle_torque =
      UnwrappedAxis(data, throttle_torque_high_, throttle_torque_low_) / 2.0;

  // TODO(austin): Deal with haptics here.
  if (throttle < 0) {
    throttle = ::std::max(-1.0, throttle / 0.7);
  }

  if (data.IsPressed(slow_down_)) {
    throttle *= 0.5;
  }

  if (!data.GetControlBit(ControlBit::kEnabled)) {
    high_gear_ = default_high_gear_;
  }

  if (data.PosEdge(shift_low_)) {
    high_gear_ = false;
  }

  if (data.PosEdge(shift_high_)) {
    high_gear_ = true;
  }

  return DrivetrainInputReader::WheelAndThrottle{
      wheel,    wheel_velocity,    wheel_torque,
      throttle, throttle_velocity, throttle_torque,
      high_gear_};
}

DrivetrainInputReader::WheelAndThrottle
XboxDrivetrainInputReader::GetWheelAndThrottle(
    const ::aos::input::driver_station::Data &data) {
  // xbox
  constexpr double kWheelDeadband = 0.05;
  constexpr double kThrottleDeadband = 0.05;
  const double wheel =
      aos::Deadband(-data.GetAxis(wheel_), kWheelDeadband, 1.0);

  const double unmodified_throttle =
      aos::Deadband(-data.GetAxis(throttle_), kThrottleDeadband, 1.0);

  // Apply a sin function that's scaled to make it feel better.
  constexpr double throttle_range = M_PI_2 * 0.9;

  double throttle = ::std::sin(throttle_range * unmodified_throttle) /
                    ::std::sin(throttle_range);
  throttle = ::std::sin(throttle_range * throttle) / ::std::sin(throttle_range);
  throttle = 2.0 * unmodified_throttle - throttle;
  return DrivetrainInputReader::WheelAndThrottle{wheel, 0.0, 0.0, throttle,
                                                 0.0,   0.0, true};
}

std::unique_ptr<SteeringWheelDrivetrainInputReader>
SteeringWheelDrivetrainInputReader::Make(::aos::EventLoop *event_loop,
                                         bool default_high_gear) {
  const JoystickAxis kSteeringWheel(1, 1), kDriveThrottle(2, 2);
  const ButtonLocation kQuickTurn(1, 5);
  const ButtonLocation kTurn1(1, 7);
  const ButtonLocation kTurn2(1, 11);
  std::unique_ptr<SteeringWheelDrivetrainInputReader> result(
      new SteeringWheelDrivetrainInputReader(
          event_loop, kSteeringWheel, kDriveThrottle, kQuickTurn, kTurn1,
          TurnButtonUse::kControlLoopDriving, kTurn2,
          TurnButtonUse::kControlLoopDriving));
  result.get()->set_default_high_gear(default_high_gear);

  return result;
}

std::unique_ptr<PistolDrivetrainInputReader> PistolDrivetrainInputReader::Make(
    ::aos::EventLoop *event_loop, bool default_high_gear,
    TopButtonUse top_button_use) {
  // Pistol Grip controller
  const JoystickAxis kTriggerHigh(1, 1), kTriggerLow(1, 4),
      kTriggerVelocityHigh(1, 2), kTriggerVelocityLow(1, 5),
      kTriggerTorqueHigh(1, 3), kTriggerTorqueLow(1, 6);

  const JoystickAxis kWheelHigh(2, 1), kWheelLow(2, 4),
      kWheelVelocityHigh(2, 2), kWheelVelocityLow(2, 5), kWheelTorqueHigh(2, 3),
      kWheelTorqueLow(2, 6);

  const ButtonLocation kQuickTurn(1, 3);

  const ButtonLocation TopButton(1, 1);
  const ButtonLocation SecondButton(1, 2);
  const ButtonLocation BottomButton(1, 4);
  // Non-existant button for nops.
  const ButtonLocation DummyButton(1, 10);

  // TODO(james): Make a copy assignment operator for ButtonLocation so we don't
  // have to shoehorn in these ternary operators.
  const ButtonLocation kTurn1 = (top_button_use == TopButtonUse::kLineFollow)
                                    ? SecondButton
                                    : DummyButton;
  // Turn2 does closed loop driving.
  const ButtonLocation kTurn2 =
      (top_button_use == TopButtonUse::kLineFollow) ? TopButton : DummyButton;

  const ButtonLocation kShiftHigh =
      (top_button_use == TopButtonUse::kShift) ? TopButton : DummyButton;
  const ButtonLocation kShiftLow =
      (top_button_use == TopButtonUse::kShift) ? SecondButton : DummyButton;

  std::unique_ptr<PistolDrivetrainInputReader> result(
      new PistolDrivetrainInputReader(
          event_loop, kWheelHigh, kWheelLow, kTriggerVelocityHigh,
          kTriggerVelocityLow, kTriggerTorqueHigh, kTriggerTorqueLow,
          kTriggerHigh, kTriggerLow, kWheelVelocityHigh, kWheelVelocityLow,
          kWheelTorqueHigh, kWheelTorqueLow, kQuickTurn, kShiftHigh, kShiftLow,
          kTurn1, kTurn2, BottomButton));

  result->set_default_high_gear(default_high_gear);
  return result;
}

std::unique_ptr<XboxDrivetrainInputReader> XboxDrivetrainInputReader::Make(
    ::aos::EventLoop *event_loop) {
  // xbox
  const JoystickAxis kSteeringWheel(1, 5), kDriveThrottle(1, 2);
  const ButtonLocation kQuickTurn(1, 5);

  // Nop
  const ButtonLocation kTurn1(1, 1);
  const ButtonLocation kTurn2(1, 2);

  std::unique_ptr<XboxDrivetrainInputReader> result(
      new XboxDrivetrainInputReader(event_loop, kSteeringWheel, kDriveThrottle,
                                    kQuickTurn, kTurn1,
                                    TurnButtonUse::kControlLoopDriving, kTurn2,
                                    TurnButtonUse::kControlLoopDriving));
  return result;
}
::std::unique_ptr<DrivetrainInputReader> DrivetrainInputReader::Make(
    ::aos::EventLoop *event_loop, InputType type,
    const drivetrain::DrivetrainConfig<double> &dt_config) {
  std::unique_ptr<DrivetrainInputReader> drivetrain_input_reader;

  using InputType = DrivetrainInputReader::InputType;

  switch (type) {
    case InputType::kSteeringWheel:
      drivetrain_input_reader = SteeringWheelDrivetrainInputReader::Make(
          event_loop, dt_config.default_high_gear);
      break;
    case InputType::kPistol:
      drivetrain_input_reader = PistolDrivetrainInputReader::Make(
          event_loop, dt_config.default_high_gear,
          dt_config.pistol_grip_shift_enables_line_follow
              ? PistolDrivetrainInputReader::TopButtonUse::kLineFollow
              : PistolDrivetrainInputReader::TopButtonUse::kShift);
      break;
    case InputType::kXbox:
      drivetrain_input_reader = XboxDrivetrainInputReader::Make(event_loop);
      break;
  }
  return drivetrain_input_reader;
}

}  // namespace input
}  // namespace aos
