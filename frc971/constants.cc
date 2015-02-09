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

const double kCompDrivetrainEncoderRatio =
    (18.0 / 50.0) /*output reduction*/ * (56.0 / 30.0) /*encoder gears*/;
const double kCompLowGearRatio = 18.0 / 60.0 * 18.0 / 50.0;
const double kCompHighGearRatio = 28.0 / 50.0 * 18.0 / 50.0;

const double kPracticeDrivetrainEncoderRatio = kCompDrivetrainEncoderRatio;
const double kPracticeLowGearRatio = kCompLowGearRatio;
const double kPracticeHighGearRatio = kCompHighGearRatio;

const ShifterHallEffect kCompRightDriveShifter{555, 657, 660, 560, 0.2, 0.7};
const ShifterHallEffect kCompLeftDriveShifter{555, 660, 644, 552, 0.2, 0.7};

const ShifterHallEffect kPracticeRightDriveShifter{2.95, 3.95, 3.95, 2.95, 0.2, 0.7};
const ShifterHallEffect kPracticeLeftDriveShifter{2.95, 4.2, 3.95, 3.0, 0.2, 0.7};

// TODO(sensors): Get actual robot width before turning on robot.
const double kRobotWidth = 25.0 / 100.0 * 2.54;


// ///// Superstructure Constants

// Gearing ratios of the pots and encoders for the elevator and arm.
// Ratio is output shaft rotations per encoder/pot rotation
const double kArmEncoderRatio = 18.0 / 48.0 * 16.0 / 72.0;
// const double kArmPotRatio = 48.0 / 48.0 * 16.0 / 72.0;
const double kElevatorEncoderRatio = 14.0 / 84.0;
// const double kElevatorPotRatio = 1.0;
const double kClawEncoderRatio = 18.0 / 72.0;
// const double kClawPotRatio = 18.0/72.0;

// Elevator gearbox pulley output constants.
const int kElevatorGearboxOutputPulleyTeeth = 32;  // 32 teeth
const double kElevatorGearboxOutputPitch = 0.005;  // 5 mm/tooth
const double kElevatorGearboxOutputRotationDistance =
    kElevatorGearboxOutputPulleyTeeth * kElevatorGearboxOutputPitch;

// Number of radians between each index pulse on the arm.
const double kArmEncoderIndexDifference = 2 * M_PI *  kArmEncoderRatio;

// Number of meters betwen index pulses on the elevator.
const double kElevatorEncoderIndexDifference =
    kElevatorGearboxOutputRotationDistance * kElevatorEncoderRatio;

const double kClawEncoderIndexDifference = 2.0 * M_PI * kClawEncoderRatio;

const int kZeroingSampleSize = 20;

const Values *DoGetValuesForTeam(uint16_t team) {
  switch (team) {
    case 1:  // for tests
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          0.5,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
          5.0,   // drivetrain max speed

          // Zeroing constants: left arm, right arm, left elev, right elev
          {
           kZeroingSampleSize, kArmEncoderIndexDifference,
           // TODO(sensors): Get actual offsets before turning on robot.
           0.0 /*index_offset_at_zero*/
          },
          {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kClawEncoderIndexDifference, 0.0},

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {
            // Claw values, in radians.
            // 0 is level with the ground.
            // Positive moves in the direction of positive encoder values.
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
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
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
          }
          // End "sensor" values.
      };
      break;
    case kCompTeamNumber:
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
          5.0,   // drivetrain max speed

          // Zeroing constants: left arm, right arm, left elev, right elev
          {
           kZeroingSampleSize, kArmEncoderIndexDifference,
           // TODO(sensors): Get actual offsets before turning on robot.
           0.0 /*index_offset_at_zero*/
          },
          {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kClawEncoderIndexDifference, 0.0},

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {
            // Claw values, in radians.
            // 0 is level with the ground.
            // Positive moves in the direction of positive encoder values.
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
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
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
          }
          // End "sensor" values.
      };
      break;
    case kPracticeTeamNumber:
      return new Values{
          kPracticeDrivetrainEncoderRatio,
          kPracticeLowGearRatio,
          kPracticeHighGearRatio,
          kPracticeLeftDriveShifter,
          kPracticeRightDriveShifter,
          false,
          kRobotWidth,
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          0.02,  // drivetrain done delta
          5.0,   // drivetrain max speed

          // Zeroing constants: left arm, right arm, left elev, right elev
          {
           kZeroingSampleSize, kArmEncoderIndexDifference,
           // TODO(sensors): Get actual offsets before turning on robot.
           0.0 /*index_offset_at_zero*/
          },
          {kZeroingSampleSize, kArmEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kElevatorEncoderIndexDifference, 0.0},
          {kZeroingSampleSize, kClawEncoderIndexDifference, 0.0},
          // TODO(sensors): End "sensors" values

          // Motion ranges: hard_lower_limit, hard_upper_limit,
          //                soft_lower_limit, soft_upper_limit
          // TODO(sensors): Get actual bounds before turning on robot.
          {
            // Claw values, in radians.
            // 0 is level with the ground.
            // Positive moves in the direction of positive encoder values.
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
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
            {0.0000000000, 1.5700000000,
             0.1000000000, 1.2000000000}
          }
          // TODO(sensors): End "sensor" values.
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
