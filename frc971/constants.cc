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

const ShifterHallEffect kPracticeLeftDriveShifter{5, 0, 0.60,
                                                  0.47};
const ShifterHallEffect kPracticeRightDriveShifter{5, 0, 0.62,
                                                   0.55};
const double shooter_zeroing_off_speed = 0.0;
const double shooter_zeroing_speed = 0.1;

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
           {-2, 0.001786, 0.001786, -2},
           {-2, -0.000446, -2, 0.026938},
           {0.005358, 0.014436, 0.014436, 0.026491},
           shooter_zeroing_off_speed,
           shooter_zeroing_speed
          },
          {0.200000,
          0.100000,
          0.000000,
          -0.762218,
          0.912207,
          -0.362218,
          0.512207,
          {-1.682379, 1.043334, -1.282379, 0.643334, {-1.7, -1.544662, -1.7, -1.547616}, {-0.130218, -0.019771, -0.132036, -0.018862}, {0.935842, 1.1, 0.932660, 1.1}},
          {-1.225821, 1.553752, -0.825821, 1.153752, {-1.3, -1.088331, -1.3, -1.088331}, {-0.134536, -0.018408, -0.136127, -0.019771}, {1.447396, 1.6, 1.443987, 1.6}},
          0.020000,  // claw_unimportant_epsilon
          -0.200000,   // start_fine_tune_pos
          4.000000,
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
