#include "y2015/constants.h"

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/network/team_number.h"
#include "aos/common/mutex.h"

#include "y2015/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "y2015/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace frc971 {
namespace constants {
namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

// ///// Drivetrain Constants

// These three constants were set by Daniel on 2/13/15.
const double kDrivetrainEncoderRatio = 20.0 / 50.0;
const double kLowGearRatio = kDrivetrainEncoderRatio * 20.0 / 50.0;
const double kHighGearRatio = kLowGearRatio;

const ShifterHallEffect kCompRightDriveShifter{555, 657, 660, 560, 0.2, 0.7};
const ShifterHallEffect kCompLeftDriveShifter{555, 660, 644, 552, 0.2, 0.7};

const ShifterHallEffect kPracticeRightDriveShifter{2.95, 3.95, 3.95,
                                                   2.95, 0.2,  0.7};
const ShifterHallEffect kPracticeLeftDriveShifter{2.95, 4.2, 3.95,
                                                  3.0,  0.2, 0.7};
const double kToteHeight = 0.3;

// Set by Daniel on 2/13/15.
// Distance from the center of the left wheel to the center of the right wheel.
const double kRobotWidth = 37.806 /*inches*/ * 0.0254;

// ///// Superstructure Constants

// Elevator gearbox pulley output constants.
const int kElevatorGearboxOutputPulleyTeeth = 32;  // 32 teeth
const double kElevatorGearboxOutputPitch = 0.005;  // 5 mm/tooth
const double kElevatorGearboxOutputRadianDistance =
    kElevatorGearboxOutputPulleyTeeth * kElevatorGearboxOutputPitch /
    (2.0 * M_PI);

const double kArmZeroingHeight = 0.2;

const double kMaxAllowedLeftRightArmDifference = 0.12;  // radians
const double kMaxAllowedLeftRightElevatorDifference = 0.04;  // meters

const Values::ClawGeometry kClawGeometry{
    // Horizontal distance from the center of the grabber to the end.
    0.5 * 18.0 * 0.0254,
    // Vertical distance from the arm rotation center to the bottom of
    // the
    // grabber.  Distance measured with arm vertical (theta = 0).
    5.1 * 0.0254,
    // Vertical separation of the claw and arm rotation centers with the
    // elevator at 0.0 and the arm angle set to zero.
    10.5 * 0.0254,
    // Horizontal separation of the claw and arm rotation centers with
    // the
    // elevator at 0.0 and the arm angle set to zero.
    6.5 * 0.0254,
    // Distance between the center of the claw to the top of the claw.
    // 2.75 inches would work most of the time.  Using 3.5 inches because
    // of the
    // pnumatics fitting on the piston.
    3.5 * 0.0254,
    // The grabber is safe at any height if it is behind this location.
    // The location of the grabber is used here and not the location of
    // the end
    // of the grabber.  The grabber location is (0, 0) when the elevator
    // is at 0
    // and the arm angle is 0.
    (18.0 - 0.3) * 0.0254,
    // The grabber is safe at any x if it is above this location.
    // The location of the grabber is used here and not the location of
    // the end
    // of the grabber.  The grabber location is (0, 0) when the elevator
    // is at 0
    // and the arm angle is 0.
    // The "-5.4" is the location of the bottom of the grabber when
    // the elevator is at the bottom (-0.3 inches) and arm angle is 0.
    -8.0 * 0.0254,
};

// Gearing ratios of the pots and encoders for the elevator and arm.
// Ratio is output shaft rotations per encoder/pot rotation
// Checked by Daniel on 2/13/15.
const double kArmEncoderRatio = 18.0 / 48.0 * 16.0 / 72.0;
const double kArmPotRatio = 48.0 / 48.0 * 16.0 / 72.0;
const double kElevatorEncoderRatio = 14.0 / 84.0;
const double kElevatorPotRatio = 1.0;
const double kClawEncoderRatio = 18.0 / 72.0;
const double kClawPotRatio = 18.0 / 72.0;

// Number of radians between each index pulse on the arm.
const double kArmEncoderIndexDifference = 2.0 * M_PI * kArmEncoderRatio;
// Number of meters between each index pulse on the elevator.
const double kElevatorEncoderIndexDifference =
    kElevatorEncoderRatio * 2.0 * M_PI *  // radians
    kElevatorGearboxOutputRadianDistance;
// Number of radians between index pulses on the claw.
const double kClawEncoderIndexDifference = 2.0 * M_PI * kClawEncoderRatio;

const int kZeroingSampleSize = 200;

// The length of the arm.
const double kArmLength = 0.7366;

// TODO(danielp): All these values might need to change.
const double kClawPistonSwitchTime = 0.4;
const double kClawZeroingRange = 0.3;

const Values *DoGetValuesForTeam(uint16_t team) {
  switch (team) {
    case 1:  // for tests
      return new Values{
          kDrivetrainEncoderRatio,
          kArmEncoderRatio,
          kArmPotRatio,
          kElevatorEncoderRatio,
          kElevatorPotRatio,
          kElevatorGearboxOutputRadianDistance,
          kClawEncoderRatio,
          kClawPotRatio,
          kToteHeight,
          kLowGearRatio,
          kHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          0.5,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          5.0,   // drivetrain max speed

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          {// Claw values, in radians.
           // 0 is level with the ground.
           // Positive moves in the direction of positive encoder values.
           {-0.05, M_PI / 2.0 + 0.1, 0.0, M_PI / 2.0},

           // Zeroing constants for wrist.
           {kZeroingSampleSize, kClawEncoderIndexDifference, 0.9, 0.3},

           6.308141,
           kClawPistonSwitchTime,
           kClawZeroingRange},

          {// Elevator values, in meters.
           // 0 is the portion of the elevator carriage that Spencer removed
           // lining up with the bolt.
           // Positive is up.
           {-0.005, 0.689000, 0.010000, 0.680000},

           // Arm values, in radians.
           // 0 is sticking straight out horizontally over the intake/front.
           // Positive is rotating up and into the robot (towards the back).
           {-M_PI / 2 - 0.05, M_PI / 2 + 0.05, -M_PI / 2, M_PI / 2},

           // Elevator zeroing constants: left, right.
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0, 0.3},
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0, 0.3},
           // Arm zeroing constants: left, right.
           {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0, 0.3},
           {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0, 0.3},
           0.7,
           -0.08,
           -3.5 - 0.01 - -0.02,
           3.5 - 0.17 - -0.15,

           kArmZeroingHeight,
           kArmLength,
          },
          // End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
          kClawGeometry,
      };
      break;
    case kPracticeTeamNumber:
      return new Values{
          kDrivetrainEncoderRatio,
          kArmEncoderRatio,
          kArmPotRatio,
          kElevatorEncoderRatio,
          kElevatorPotRatio,
          kElevatorGearboxOutputRadianDistance,
          kClawEncoderRatio,
          kClawPotRatio,
          kToteHeight,
          kLowGearRatio,
          kHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          5.0,   // drivetrain max speed

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {// Claw values, in radians.
           // 0 is level with the ground.
           // Positive moves in the direction of positive encoder values.
           {-0.05, M_PI / 2.0 + 0.1, 0.0, M_PI / 2.0},

           // Zeroing constants for wrist.
           {kZeroingSampleSize, kClawEncoderIndexDifference, 0.9104180000000001,
            0.3},

           6.308141,
           kClawPistonSwitchTime,
           kClawZeroingRange},

          {// Elevator values, in meters.
           // 0 is the portion of the elevator carriage that Spencer removed
           // lining up with the bolt.
           // Positive is up.
           {-0.00500, 0.689000, 0.010000, 0.680000},

           // Arm values, in radians.
           // 0 is sticking straight out horizontally over the intake/front.
           // Positive is rotating up and into the robot (towards the back).
           {-M_PI / 2 - 0.05, M_PI / 2 + 0.05, -M_PI / 2, M_PI / 2},

           // Elevator zeroing constants: left, right.
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.110677, 0.3},  // Was 0.088984 (3 mm too high)
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.109974, 0.3},  // Was 0.104557 (4 mm too low)
           // Arm zeroing constants: left, right.
           {kZeroingSampleSize, kArmEncoderIndexDifference, -0.324437, 0.3},
           {kZeroingSampleSize, kArmEncoderIndexDifference, -0.064683, 0.3},
           0.722230 - -0.000594 - -0.026183 - 0.003442,   // Left Elevator Poteniometer adjustment
           -0.081354 - -0.000374 - -0.024793 - -0.006916, // Right Elevator Poteniometer adjustment
           -3.509611 - 0.007415 - -0.019081,
           3.506927 - 0.170017 - -0.147970,

           kArmZeroingHeight,
           kArmLength,
          },
          // End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
          kClawGeometry,
      };
      break;
    case kCompTeamNumber:
      return new Values{
          kDrivetrainEncoderRatio,
          kArmEncoderRatio,
          kArmPotRatio,
          kElevatorEncoderRatio,
          kElevatorPotRatio,
          kElevatorGearboxOutputRadianDistance,
          kClawEncoderRatio,
          kClawPotRatio,
          kToteHeight,
          kLowGearRatio,
          kHighGearRatio,
          kPracticeLeftDriveShifter,
          kPracticeRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          5.0,   // drivetrain max speed

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {// Claw values, in radians.
           // 0 is level with the ground.
           // Positive moves in the direction of positive encoder values.
           {-0.05, M_PI / 2.0 + 0.1, 0.0, M_PI / 2.0},

           // Zeroing constants for wrist.
           // TODO(sensors): Get actual offsets for these.
           {kZeroingSampleSize, kClawEncoderIndexDifference, 0.952602, 0.3},
           6.1663463999999992 + 0.015241,

           kClawPistonSwitchTime,
           kClawZeroingRange},

          {// Elevator values, in meters.
           // 0 is at the top of the elevator frame.
           // Positive is down towards the drivebase.
           {-0.00500, 0.689000, 0.010000, 0.680000},

           // Arm values, in radians.
           // 0 is sticking straight out horizontally over the intake/front.
           // Positive is rotating up and into the robot (towards the back).
           {-M_PI / 2 - 0.05, M_PI / 2 + 0.05, -M_PI / 2, M_PI / 2},

           // Elevator zeroing constants: left, right.
           // These are the encoder offsets.
           // TODO(sensors): Get actual offsets for these.
           {kZeroingSampleSize, kElevatorEncoderIndexDifference,
            0.240286 - 0.007969 - 0.025, 0.3},
           {kZeroingSampleSize, kElevatorEncoderIndexDifference,
            0.234583 - 0.000143, 0.3},
           // Arm zeroing constants: left, right.
           {kZeroingSampleSize, kArmEncoderIndexDifference, 0.060592, 0.3},
           {kZeroingSampleSize, kArmEncoderIndexDifference, 0.210155, 0.3},
           // These are the potentiometer offsets.
           0.72069366666666679 - 0.026008 - 0.024948 + 0.025,
           -0.078959636363636357 - 0.024646  - 0.020260,
           -3.509611 - 0.007415 - -0.019081 - 0.029393 - -0.013585,
           3.506927 - 0.170017 - -0.147970 - 0.005045 - -0.026504,

           kArmZeroingHeight,
           kArmLength,
          },
          // TODO(sensors): End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
          kClawGeometry,
      };
      break;
    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }
}

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  return DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static ::aos::Once<const Values> once(DoGetValues);
  return *once.Get();
}

const Values &GetValuesForTeam(uint16_t team_number) {
  static ::aos::Mutex mutex;
  ::aos::MutexLocker locker(&mutex);

  // IMPORTANT: This declaration has to stay after the mutex is locked to avoid
  // race conditions.
  static ::std::map<uint16_t, const Values *> values;

  if (values.count(team_number) == 0) {
    values[team_number] = DoGetValuesForTeam(team_number);
#if __has_feature(address_sanitizer)
    __lsan_ignore_object(values[team_number]);
#endif
  }
  return *values[team_number];
}

}  // namespace constants
}  // namespace frc971
