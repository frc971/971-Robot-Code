#ifndef AOS_INPUT_DRIVETRAIN_INPUT_H_
#define AOS_INPUT_DRIVETRAIN_INPUT_H_

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <memory>

#include "aos/common/input/driver_station_data.h"
#include "aos/common/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace aos {
namespace input {

// We have a couple different joystick configurations used to drive our skid
// steer robots.  These configurations don't change very often, and there are a
// small, discrete, set of them.  The interface to the drivetrain is the same
// across all of our drivetrains, so we can abstract that away from our users.
//
// We want to be able to re-use that code across years, and switch joystick
// types quickly and efficiently.
//
// DrivetrainInputReader is the abstract base class which provides a consistent
// API to control drivetrains.
//
// To construct the appropriate DrivetrainInputReader, call
// DrivetrainInputReader::Make with the input type enum.
// To use it, call HandleDrivetrain(data) every time a joystick packet is
// received.
//
// Base class for the interface to the drivetrain implemented by multiple
// joystick types.
class DrivetrainInputReader {
 public:
  // Inputs driver station button and joystick locations
  DrivetrainInputReader(driver_station::JoystickAxis kSteeringWheel,
                        driver_station::JoystickAxis kDriveThrottle,
                        driver_station::ButtonLocation kQuickTurn,
                        driver_station::ButtonLocation kTurn1,
                        driver_station::ButtonLocation kTurn2)
      : kSteeringWheel(kSteeringWheel),
        kDriveThrottle(kDriveThrottle),
        kQuickTurn(kQuickTurn),
        kTurn1(kTurn1),
        kTurn2(kTurn2) {}

  virtual ~DrivetrainInputReader() = default;

  // List of driver joystick types.
  enum class InputType {
    kSteeringWheel,
    kPistol,
    kXbox,
  };

  // Constructs the appropriate DrivetrainInputReader.
  static std::unique_ptr<DrivetrainInputReader> Make(InputType type);

  // Processes new joystick data and publishes drivetrain goal messages.
  void HandleDrivetrain(const ::aos::input::driver_station::Data &data);

  // Sets the scalar for the steering wheel for closed loop mode converting
  // steering ratio to meters displacement on the two wheels.
  void set_wheel_multiplier(double wheel_multiplier) {
    wheel_multiplier_ = wheel_multiplier;
  }

  // Returns the current robot velocity in m/s.
  double robot_velocity() const { return robot_velocity_; }

 protected:
  const driver_station::JoystickAxis kSteeringWheel;
  const driver_station::JoystickAxis kDriveThrottle;
  const driver_station::ButtonLocation kQuickTurn;
  const driver_station::ButtonLocation kTurn1;
  const driver_station::ButtonLocation kTurn2;

  // Structure containing the (potentially adjusted) steering and throttle
  // values from the joysticks.
  struct WheelAndThrottle {
    double wheel;
    double throttle;
  };

 private:
  // Computes the steering and throttle from the provided driverstation data.
  virtual WheelAndThrottle GetWheelAndThrottle(
      const ::aos::input::driver_station::Data &data) = 0;

  double robot_velocity_ = 0.0;
  // Goals to send to the drivetrain in closed loop mode.
  double left_goal_ = 0.0;
  double right_goal_ = 0.0;
  // The scale for the joysticks for closed loop mode converting
  // joysticks to meters displacement on the two wheels.
  double wheel_multiplier_ = 0.5;
};

// Implements DrivetrainInputReader for the original steering wheel.
class SteeringWheelDrivetrainInputReader : public DrivetrainInputReader {
 public:
  using DrivetrainInputReader::DrivetrainInputReader;

  // Creates a DrivetrainInputReader with the corresponding joystick ports and
  // axis for the big steering wheel and throttle stick.
  static std::unique_ptr<SteeringWheelDrivetrainInputReader> Make();

 private:
  WheelAndThrottle GetWheelAndThrottle(
      const ::aos::input::driver_station::Data &data) override;
};

class PistolDrivetrainInputReader : public DrivetrainInputReader {
 public:
  using DrivetrainInputReader::DrivetrainInputReader;

  // Creates a DrivetrainInputReader with the corresponding joystick ports and
  // axis for the (cheap) pistol grip controller.
  static std::unique_ptr<PistolDrivetrainInputReader> Make();

 private:
  WheelAndThrottle GetWheelAndThrottle(
      const ::aos::input::driver_station::Data &data) override;
};

class XboxDrivetrainInputReader : public DrivetrainInputReader {
 public:
  using DrivetrainInputReader::DrivetrainInputReader;

  // Creates a DrivetrainInputReader with the corresponding joystick ports and
  // axis for the Xbox controller.
  static std::unique_ptr<XboxDrivetrainInputReader> Make();

 private:
  WheelAndThrottle GetWheelAndThrottle(
      const ::aos::input::driver_station::Data &data) override;
};

}  // namespace input
}  // namespace aos

#endif  // AOS_INPUT_DRIVETRAIN_INPUT_H_
