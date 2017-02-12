#include "y2017/constants.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"
#include "aos/common/network/team_number.h"
#include "aos/common/once.h"

#include "y2017/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2017/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2017 {
namespace constants {

const int Values::kZeroingSampleSize;

constexpr double Values::kDrivetrainEncoderRatio;

constexpr double Values::kShooterEncoderRatio;

constexpr double Values::kIntakeEncoderRatio, Values::kIntakePotRatio,
    Values::kIntakeEncoderIndexDifference;
constexpr ::frc971::constants::Range Values::kIntakeRange;

constexpr double Values::kHoodEncoderRatio, Values::kHoodPotRatio,
    Values::kHoodEncoderIndexDifference;
constexpr ::frc971::constants::Range Values::kHoodRange;

constexpr double Values::kTurretEncoderRatio, Values::kTurretPotRatio,
    Values::kTurretEncoderIndexDifference;
constexpr ::frc971::constants::Range Values::kTurretRange;

constexpr double Values::kIndexerEncoderRatio,
    Values::kIndexerEncoderIndexDifference;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  Values::Intake *const intake = &r->intake;
  Values::Turret *const turret = &r->turret;
  Values::Hood *const hood = &r->hood;

  r->drivetrain_max_speed = 5;

  intake->zeroing.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing.index_difference = Values::kIntakeEncoderIndexDifference;
  intake->zeroing.measured_index_position = 0;
  intake->zeroing.allowable_encoder_error = 0.3;

  turret->zeroing.average_filter_size = Values::kZeroingSampleSize;
  turret->zeroing.index_difference = Values::kTurretEncoderIndexDifference;
  turret->zeroing.measured_index_position = 0;
  turret->zeroing.allowable_encoder_error = 0.3;

  hood->zeroing.average_filter_size = Values::kZeroingSampleSize;
  hood->zeroing.index_difference = Values::kHoodEncoderIndexDifference;
  hood->zeroing.measured_index_position = 0.1;
  hood->zeroing.allowable_encoder_error = 0.3;

  switch (team) {
    // A set of constants for tests.
    case 1:
      intake->pot_offset = 0;
      turret->pot_offset = 0;
      hood->pot_offset = 0.1;
      r->down_error = 0;
      r->vision_name = "test";
      break;

    case kCompTeamNumber:
      intake->pot_offset = 0;
      turret->pot_offset = 0;
      hood->pot_offset = 0.1;
      r->down_error = 0;
      r->vision_name = "competition";
      break;

    case kPracticeTeamNumber:
      intake->pot_offset = 0;
      turret->pot_offset = 0;
      hood->pot_offset = 0.1;
      r->down_error = 0;
      r->vision_name = "practice";
      break;

    default:
      LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
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
}  // namespace y2017
