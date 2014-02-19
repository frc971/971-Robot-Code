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

const ShifterHallEffect kCompLeftDriveShifter{0.83, 2.32, 1.2, 1.0};
const ShifterHallEffect kCompRightDriveShifter{0.865, 2.375, 1.2, 1.0};

const ShifterHallEffect kPracticeLeftDriveShifter{2.082283, 0.834433, 0.60,
                                                  0.47};
const ShifterHallEffect kPracticeRightDriveShifter{2.070124, 0.838993, 0.62,
                                                   0.55};
const double shooter_zeroing_off_speed=0.0;
const double shooter_zeroing_speed=1.0;

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  switch (team) {
    case 1:  // for tests
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          true,
          control_loops::MakeVClutchDrivetrainLoop,
          control_loops::MakeClutchDrivetrainLoop,
          // ShooterLimits
          // TODO(ben): make these real numbers
          {-0.00127, 0.298196, -0.001524, 0.305054, 0.0149098,
           {-0.001778, 0.000762, 0, 0}, {-0.001778, 0.009906, 0, 0}, {0.006096, 0.026416, 0, 0},
           shooter_zeroing_off_speed,
           shooter_zeroing_speed
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
          }
      };
      break;
    case kCompTeamNumber:
      return new Values{
          kCompDrivetrainEncoderRatio,
          kCompLowGearRatio,
          kCompHighGearRatio,
          kCompLeftDriveShifter,
          kCompRightDriveShifter,
          true,
          control_loops::MakeVClutchDrivetrainLoop,
          control_loops::MakeClutchDrivetrainLoop,
          // ShooterLimits
          // TODO(ben): make these real numbers
          {-0.00127, 0.298196, -0.001524, 0.305054, 0.0149098,
           {-0.001778, 0.000762, 0, 0}, {-0.001778, 0.009906, 0, 0}, {0.006096, 0.026416, 0, 0},
           shooter_zeroing_off_speed,
           shooter_zeroing_speed
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
          }
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
          // TODO(ben): make these real numbers
          {-0.000446, 0.300038, -0.001, 0.304354,
            0.014436,
           {-2, 0.001786, 0.001786, -2}, {-2, -0.000446, -2, 0.026938}, {0.006096, 0.026416, 0, 0},
           shooter_zeroing_off_speed,
           shooter_zeroing_speed
          },
          {0.5,
           0.2,
           0.1,
           -0.446558,
           0.90675,
           -0.39110,
           0.843349,
#if 0
         separations (top, bottom)
           hard min position:-0.253845, position:-0.001136,
           soft min position:-0.244528, position:-0.047269,
           soft max position:0.526326, position:-0.510872,
           hard max position:0.517917, position:-0.582685,
#endif
           {-1.62102, 1.039699, -1.606248, 0.989702, {-1.65, -1.546252, -1.65, -1.548752}, {-0.13249, -0.02113, -0.134763, -0.021589}, {0.934024, 1.05, 0.92970, 1.05}},
           {-1.420352, 1.348313, -1.161281, 1.264001, {-1.45, -1.283771, -1.45, -1.28468}, {-0.332476, -0.214984, -0.334294, -0.217029}, {1.248547, 1.37, 1.245366, 1.37}},
           0.01,  // claw_unimportant_epsilon
           0.9,  // start_fine_tune_pos
           4.0,
          }
      };
      break;
    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }
}

}  // namespace

const Values &GetValues() {
  static ::aos::Once<const Values> once(DoGetValues);
  return *once.Get();
}

}  // namespace constants
}  // namespace frc971
