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

constexpr double Values::kDrivetrainCyclesPerRevolution,
    Values::kDrivetrainEncoderCountsPerRevolution,
    Values::kDrivetrainEncoderRatio,
    Values::kMaxDrivetrainEncoderPulsesPerSecond;

constexpr double Values::kShooterEncoderCountsPerRevolution,
    Values::kShooterEncoderRatio, Values::kMaxShooterEncoderPulsesPerSecond;

constexpr double Values::kIntakeEncoderCountsPerRevolution,
    Values::kIntakeEncoderRatio, Values::kIntakePotRatio,
    Values::kIntakeEncoderIndexDifference,
    Values::kMaxIntakeEncoderPulsesPerSecond;
constexpr ::frc971::constants::Range Values::kIntakeRange;

constexpr double Values::kHoodEncoderCountsPerRevolution,
    Values::kHoodEncoderRatio, Values::kHoodEncoderIndexDifference,
    Values::kMaxHoodEncoderPulsesPerSecond;
constexpr ::frc971::constants::Range Values::kHoodRange;

constexpr double Values::kTurretEncoderCountsPerRevolution,
    Values::kTurretEncoderRatio, Values::kMaxTurretEncoderPulsesPerSecond;

constexpr double Values::kIndexerEncoderCountsPerRevolution,
    Values::kIndexerEncoderRatio, Values::kIndexerEncoderIndexDifference,
    Values::kMaxIndexerEncoderPulsesPerSecond;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  Values::Intake *const intake = &r->intake;
  Values::Hood *const hood = &r->hood;

  r->drivetrain_max_speed = 5;

  intake->zeroing.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing.one_revolution_distance = Values::kIntakeEncoderIndexDifference;
  intake->zeroing.zeroing_threshold = 0.0005;
  intake->zeroing.moving_buffer_size = 20;
  intake->zeroing.allowable_encoder_error = 0.3;

  hood->zeroing.index_pulse_count = 2;
  hood->zeroing.index_difference = Values::kHoodEncoderIndexDifference;
  hood->zeroing.known_index_pulse = 0;

  switch (team) {
    // A set of constants for tests.
    case 1:
      intake->pot_offset = 0;
      intake->zeroing.measured_absolute_position = 0;

      hood->pot_offset = 0.1;
      hood->zeroing.measured_index_position = 0.05;

      r->down_error = 0;
      r->vision_name = "test";
      break;

    case kCompTeamNumber:
      intake->pot_offset = 0.26712;
      intake->zeroing.measured_absolute_position = 0.008913;

      hood->zeroing.measured_index_position = 0.652898 - 0.488117;

      r->down_error = 0;
      r->vision_name = "competition";
      break;

    case kPracticeTeamNumber:
      intake->pot_offset = 0.2921 + 0.00039 + 0.012236 - 0.023602;
      intake->zeroing.measured_absolute_position = 0.031437;

      hood->zeroing.measured_index_position = 0.655432 - 0.460505;

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
