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

const Values *DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  switch (team) {
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
          0.5,
          0.1,
          0.1,
          0.0,
          1.57,
          {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05}, {1.0, 1.1}, {2.0, 2.1}},
          {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05}, {1.0, 1.1}, {2.0, 2.1}},
          0.01,  // claw_unimportant_epsilon
          0.9,   // start_fine_tune_pos
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
          0.5,
          0.2,
          0.1,
          0.0,
          1.57,
          {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05}, {1.0, 1.1}, {2.0, 2.1}},
          {0.0, 2.05, 0.02, 2.02, {-0.1, 0.05}, {1.0, 1.1}, {2.0, 2.1}},
          0.01,  // claw_unimportant_epsilon
          0.9,  // start_fine_tune_pos
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
