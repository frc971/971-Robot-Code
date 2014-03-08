#include "frc971/constants.h"

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/network/team_number.h"

#include "frc971/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/polydrivetrain_clutch_motor_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "frc971/control_loops/drivetrain/drivetrain_clutch_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace frc971 {
namespace constants {
namespace {

const double kCompDrivetrainEncoderRatio =
    (15.0 / 50.0) /*output reduction*/ * (36.0 / 24.0) /*encoder gears*/;
const double kCompLowGearRatio = 14.0 / 60.0 * 15.0 / 50.0;
const double kCompHighGearRatio = 30.0 / 44.0 * 15.0 / 50.0;

const double kPracticeDrivetrainEncoderRatio =
    (17.0 / 50.0) /*output reduction*/ * (64.0 / 24.0) /*encoder gears*/;
const double kPracticeLowGearRatio = 16.0 / 60.0 * 19.0 / 50.0;
const double kPracticeHighGearRatio = 28.0 / 48.0 * 19.0 / 50.0;

const ShifterHallEffect kCompRightDriveShifter{525, 635, 603, 529, 0.3, 0.7};
const ShifterHallEffect kCompLeftDriveShifter{525, 645, 620, 533, 0.3, 0.7};

const ShifterHallEffect kPracticeRightDriveShifter{550, 640, 635, 550, 0.2, 0.7};
const ShifterHallEffect kPracticeLeftDriveShifter{540, 620, 640, 550, 0.2, 0.7};
const double shooter_zeroing_speed = 0.05;
const double shooter_unload_speed = 0.08;

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
          control_loops::MakeVClutchDrivetrainLoop,
          control_loops::MakeClutchDrivetrainLoop,
          // ShooterLimits
          // TODO(ben): make these real numbers
          {-0.00127, 0.298196, -0.001524, 0.305054, 0.0149098,
           {-0.001778, 0.000762, 0, 0}, {-0.001778, 0.009906, 0, 0}, {0.006096, 0.026416, 0, 0},
           shooter_zeroing_speed,
           shooter_unload_speed
          },
          {0.5,
           0.1,
           0.1,
           0.0,
           1.57,
           0,
           0,
           {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05, -0.1, 0.05}, {1.0, 1.1, 1.0, 1.1}, {2.0, 2.1, 2.0, 2.1}},
           {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05, -0.1, 0.05}, {1.0, 1.1, 1.0, 1.1}, {2.0, 2.1, 2.0, 2.1}},
           0.01,  // claw_unimportant_epsilon
           0.9,   // start_fine_tune_pos
           4.0,
          },
          {0.07, 0.15}, // shooter_action
          0.02, // drivetrain done delta
          5.0 // drivetrain max speed
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
          control_loops::MakeVClutchDrivetrainLoop,
          control_loops::MakeClutchDrivetrainLoop,
          // ShooterLimits
          // TODO(ben): make these real numbers
          {-0.00127, 0.298196, -0.001524, 0.305054, 0.0149098,
           {-0.001778, 0.000762, 0, 0}, {-0.001778, 0.009906, 0, 0}, {0.006096, 0.026416, 0, 0},
           shooter_zeroing_speed,
           shooter_unload_speed
          },
          {0.5,
           0.1,
           0.1,
           0.0,
           1.57,
           0,
           0,
           {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05, 0, 0}, {1.0, 1.1, 0, 0}, {2.0, 2.1, 0, 0}},
           {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05, 0, 0}, {1.0, 1.1, 0, 0}, {2.0, 2.1, 0, 0}},
           0.01,  // claw_unimportant_epsilon
           0.9,   // start_fine_tune_pos
           4.0,
          },
          //TODO(james): Get realer numbers for shooter_action.
          {0.07, 0.15}, // shooter_action
          0.02, // drivetrain done delta
          5.0 // drivetrain max speed
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
          control_loops::MakeVDogDrivetrainLoop,
          control_loops::MakeDogDrivetrainLoop,
          // ShooterLimits
          {-0.001042, 0.294084, -0.001935, 0.303460, 0.0138401,
           {-0.002, 0.000446, -0.002, 0.000446},
           {-0.002, 0.009078, -0.002, 0.009078},
           {0.003869, 0.026194, 0.003869, 0.026194},
           shooter_zeroing_speed,
           shooter_unload_speed
          },
          {0.400000 * 2.0,
          0.200000 * 2.0,
          0.000000 * 2.0,
          -0.762218 * 2.0,
          0.912207 * 2.0,
          -0.849484,
          1.42308,
          {-3.364758, 2.086668, -3.166136, 1.95, {-1.7 * 2.0, -1.544662 * 2.0, -1.7 * 2.0, -1.547616 * 2.0}, {-0.130218 * 2.0, -0.019771 * 2.0, -0.132036 * 2.0, -0.018862 * 2.0}, {0.935842 * 2.0, 1.1 * 2.0, 0.932660 * 2.0, 1.1 * 2.0}},
          {-2.451642, 3.107504, -2.273474, 2.750, {-1.3 * 2.0, -1.088331 * 2.0, -1.3 * 2.0, -1.088331 * 2.0}, {-0.134536 * 2.0, -0.018408 * 2.0, -0.136127 * 2.0, -0.019771 * 2.0}, {1.447396 * 2.0, 1.6 * 2.0, 1.443987 * 2.0, 1.6 * 2.0}},
          0.020000 * 2.0,  // claw_unimportant_epsilon
          -0.200000 * 2.0,   // start_fine_tune_pos
          4.000000,
          },
          //TODO(james): Get realer numbers for shooter_action.
          {0.07, 0.15}, // shooter_action
          0.02, // drivetrain done delta
          5.0 // drivetrain max speed
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
  return *(DoGetValuesForTeam(team_number));
}

}  // namespace constants
}  // namespace frc971
