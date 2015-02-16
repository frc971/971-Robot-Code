#include "frc971/constants.h"

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

#include "frc971/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

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
const double kDrivetrainEncoderRatio = 20.0 / 64.0;
const double kLowGearRatio = kDrivetrainEncoderRatio * 20.0 / 50.0;
const double kHighGearRatio = kLowGearRatio;

const ShifterHallEffect kCompRightDriveShifter{555, 657, 660, 560, 0.2, 0.7};
const ShifterHallEffect kCompLeftDriveShifter{555, 660, 644, 552, 0.2, 0.7};

const ShifterHallEffect kPracticeRightDriveShifter{2.95, 3.95, 3.95, 2.95, 0.2, 0.7};
const ShifterHallEffect kPracticeLeftDriveShifter{2.95, 4.2, 3.95, 3.0, 0.2, 0.7};

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

const double kMaxAllowedLeftRightArmDifference = 0.04;  // radians
const double kMaxAllowedLeftRightElevatorDifference = 0.01;  // meters

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
const double kArmEncoderIndexDifference = 2.0 * M_PI *  kArmEncoderRatio;
// Number of meters between each index pulse on the elevator.
const double kElevatorEncoderIndexDifference =
    kElevatorEncoderRatio *
    2.0 * M_PI * // radians
    kElevatorGearboxOutputRadianDistance;
// Number of radians between index pulses on the claw.
const double kClawEncoderIndexDifference = 2.0 * M_PI * kClawEncoderRatio;

const int kZeroingSampleSize = 20;

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
          kLowGearRatio,
          kHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          0.5,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
          5.0,   // drivetrain max speed

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {
            // Claw values, in radians.
            // 0 is level with the ground.
            // Positive moves in the direction of positive encoder values.
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000},

            // Zeroing constants for wrist.
            // TODO(sensors): Get actual offsets for these.
            {kZeroingSampleSize, kClawEncoderIndexDifference, 0.0},
            0.0,
          },

          {
            // Elevator values, in meters.
            // TODO(austin): Fix this.  Positive is up.
            // 0 is at the top of the elevator frame.
            // Positive is down towards the drivebase.
            {0.0000000000, 0.6790000000,
             0.2000000000, 0.6000000000},

            // Arm values, in radians.
            // 0 is sticking straight out horizontally over the intake/front.
            // Positive is rotating up and into the robot (towards the back).
            {-1.570000000, 1.5700000000,
             -1.200000000, 1.2000000000},

            // Elevator zeroing constants: left, right.
            // TODO(sensors): Get actual offsets for these.
            {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
            {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
            // Arm zeroing constants: left, right.
            {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
            {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
            0.0, 0.0, 0.0, 0.0,
          },
          // End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
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
          kLowGearRatio,
          kHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
          5.0,   // drivetrain max speed

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {
            // Claw values, in radians.
            // 0 is level with the ground.
            // Positive moves in the direction of positive encoder values.
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000},

            // Zeroing constants for wrist.
            // TODO(sensors): Get actual offsets for these.
            {kZeroingSampleSize, kClawEncoderIndexDifference, 0.0},
            0.0,
          },

          {
            // Elevator values, in meters.
            // 0 is at the top of the elevator frame.
            // Positive is down towards the drivebase.
            {0.0000000000, 0.6790000000,
             0.2000000000, 0.6000000000},

            // Arm values, in radians.
            // 0 is sticking straight out horizontally over the intake/front.
            // Positive is rotating up and into the robot (towards the back).
            {-1.570000000, 1.5700000000,
             -1.200000000, 1.2000000000},

            // Elevator zeroing constants: left, right.
            // TODO(sensors): Get actual offsets for these.
            {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
            {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
            // Arm zeroing constants: left, right.
            {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
            {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
            0.0, 0.0, 0.0, 0.0,
          },
          // End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
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
          kLowGearRatio,
          kHighGearRatio,
          kPracticeLeftDriveShifter,
          kPracticeRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
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
           {kZeroingSampleSize, kClawEncoderIndexDifference, 0.977913},
           6.1663463999999992,
          },

          {// Elevator values, in meters.
           // 0 is at the top of the elevator frame.
           // Positive is down towards the drivebase.
           {-0.00500, 0.679000, 0.010000, 0.650000},

           // Arm values, in radians.
           // 0 is sticking straight out horizontally over the intake/front.
           // Positive is rotating up and into the robot (towards the back).
           {-M_PI / 2 - 0.05, M_PI / 2 + 0.05, -M_PI / 2, M_PI / 2},

           // Elevator zeroing constants: left, right.
           // TODO(sensors): Get actual offsets for these.
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.016041 + 0.001290},
           {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.011367 + 0.003216},
           // Arm zeroing constants: left, right.
           {kZeroingSampleSize, kArmEncoderIndexDifference, -0.312677},
           {kZeroingSampleSize, kArmEncoderIndexDifference, -0.40855},
           0.72069366666666679 - 0.026008,
           -0.078959636363636357 - 0.024646,
           -3.4952331578947375 + 0.011776,
           3.5263507647058816 - 0.018921 + 0.006545,
          },
          // TODO(sensors): End "sensor" values.

          kMaxAllowedLeftRightArmDifference,
          kMaxAllowedLeftRightElevatorDifference,
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
