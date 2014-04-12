#include "frc971/constants.h"

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"
#include "aos/common/once.h"
#include "aos/common/network/team_number.h"

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

const double kCompDrivetrainEncoderRatio =
    (18.0 / 50.0) /*output reduction*/ * (56.0 / 30.0) /*encoder gears*/;
const double kCompLowGearRatio = 18.0 / 60.0 * 18.0 / 50.0;
const double kCompHighGearRatio = 28.0 / 50.0 * 18.0 / 50.0;

const double kPracticeDrivetrainEncoderRatio = kCompDrivetrainEncoderRatio;
const double kPracticeLowGearRatio = kCompLowGearRatio;
const double kPracticeHighGearRatio = kCompHighGearRatio;

const ShifterHallEffect kCompRightDriveShifter{555, 657, 660, 560, 0.2, 0.7};
const ShifterHallEffect kCompLeftDriveShifter{555, 660, 644, 552, 0.2, 0.7};

const ShifterHallEffect kPracticeRightDriveShifter{550, 640, 635, 550, 0.2, 0.7};
const ShifterHallEffect kPracticeLeftDriveShifter{540, 620, 640, 550, 0.2, 0.7};

const double shooter_zeroing_speed = 0.05;
const double shooter_unload_speed = 0.08;

// Smaller (more negative) = opening.
const double kCompTopClawOffset = -0.120;

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
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
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
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
          // ShooterLimits
          {-0.001041, 0.296019, -0.001488, 0.302717, 0.0149098,
           {-0.002, 0.000446, -0.002, 0.000446},
           {-0.002, 0.009078, -0.002, 0.009078},
           {0.003870, 0.026194, 0.003869, 0.026343},
           shooter_zeroing_speed,
           shooter_unload_speed
          },
          {0.800000,
           0.400000,
           0.000000,
           -1.220821,
           1.822142,
           -0.849484,
           1.42309,
           // 0.0371
           {-3.3284, 2.0917, -3.1661, 1.95,
             {-3.4, -2.9368 + kCompTopClawOffset, -3.4, -2.9876 + kCompTopClawOffset},
             {-0.1433 + kCompTopClawOffset, 0.0670 + kCompTopClawOffset, -0.1460 + kCompTopClawOffset, 0.0648 + kCompTopClawOffset},
             {1.9952 + kCompTopClawOffset, 2.2, 1.9898 + kCompTopClawOffset, 2.2}},
           {-2.453460, 3.082960, -2.453460, 3.082960,
             {-2.6, -2.185752, -2.6, -2.184843},
             {-0.280434, -0.049087, -0.277707, -0.047724},
             {2.892065, 3.2, 2.888429, 3.2}},
           0.040000,  // claw_unimportant_epsilon
           -0.400000,   // start_fine_tune_pos
           4.000000,
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
          control_loops::MakeVelocityDrivetrainLoop,
          control_loops::MakeDrivetrainLoop,
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
          1.767146,
          -0.849484,
          1.42308,
          {-3.364758, 2.086668, -3.166136, 1.95,
            {-1.7 * 2.0, -1.544662 * 2.0, -1.7 * 2.0, -1.547616 * 2.0},
            {-0.130218 * 2.0, -0.019771 * 2.0, -0.132036 * 2.0, -0.018862 * 2.0},
            {0.935842 * 2.0, 1.1 * 2.0, 0.932660 * 2.0, 1.1 * 2.0}},
          {-2.451642, 3.107504, -2.273474, 2.750,
            {-1.3 * 2.0, -1.088331 * 2.0, -1.3 * 2.0, -1.088331 * 2.0},
            {-0.134536 * 2.0, -0.018408 * 2.0, -0.136127 * 2.0, -0.019771 * 2.0},
            {1.447396 * 2.0, 1.6 * 2.0, 1.443987 * 2.0, 1.6 * 2.0}},
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
