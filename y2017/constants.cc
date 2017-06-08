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
constexpr ::frc971::constants::Range Values::kTurretRange;

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
  Values::Column *const column = &r->column;

  r->drivetrain_max_speed = 5;

  intake->zeroing.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing.one_revolution_distance = Values::kIntakeEncoderIndexDifference;
  intake->zeroing.zeroing_threshold = 0.0005;
  intake->zeroing.moving_buffer_size = 20;
  intake->zeroing.allowable_encoder_error = 1.9;

  column->turret_zeroed_distance = M_PI / 2.0;
  column->indexer_zeroing.index_difference = 2.0 * M_PI;
  column->indexer_zeroing.hall_trigger_zeroing_length = 2;
  column->indexer_zeroing.zeroing_move_direction = true;
  column->turret_zeroing.index_difference = 2.0 * M_PI;
  column->turret_zeroing.hall_trigger_zeroing_length = 2;
  column->turret_zeroing.zeroing_move_direction = false;

  hood->zeroing.index_pulse_count = 2;
  hood->zeroing.index_difference = Values::kHoodEncoderIndexDifference;
  hood->zeroing.known_index_pulse = 0;

  switch (team) {
    // A set of constants for tests.
    case 1:
      intake->pot_offset = 0;
      intake->zeroing.measured_absolute_position = 0;

      // TODO(austin): Swap the turret and indexer limits and make sure the
      // tests still pass.
      column->indexer_zeroing.lower_hall_position = 0.1;
      column->indexer_zeroing.upper_hall_position = 0.2;

      column->turret_zeroing.lower_hall_position = 2;
      column->turret_zeroing.upper_hall_position = 2.1;

      hood->pot_offset = 0.1;
      hood->zeroing.measured_index_position = 0.05;

      r->down_error = 0;
      r->vision_name = "test";
      r->vision_error = -0.030;
      break;

    case kCompTeamNumber:
      intake->pot_offset = 0.26712 + 0.0035 + 0.033 + 0.0011 -0.046872;
      intake->zeroing.measured_absolute_position = 0.003397;

      column->indexer_zeroing.lower_hall_position = 5.201948;
      column->indexer_zeroing.upper_hall_position = 5.508744;

      column->turret_zeroing.lower_hall_position = -4.861087;
      column->turret_zeroing.upper_hall_position = -4.680861;

      hood->zeroing.measured_index_position = 0.234766;

      r->down_error = 0;
      r->vision_name = "competition";
      r->vision_error = 0.015;
      break;

    case kPracticeTeamNumber:
      intake->pot_offset = 0.2921 + 0.00039 + 0.012236 - 0.023602 + 0.010722 +
                           0.012880 - 0.01743;
      intake->zeroing.measured_absolute_position = 0.043179;

      column->indexer_zeroing.lower_hall_position = 2.594181;
      column->indexer_zeroing.upper_hall_position = 2.886952;

      column->turret_zeroing.lower_hall_position = -4.918530;
      column->turret_zeroing.upper_hall_position = -4.720353;

      hood->zeroing.measured_index_position = 0.124275;

      r->down_error = 0;
      r->vision_name = "practice";
      r->vision_error = 0.0;
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

Values::ShotParams Values::ShotParams::BlendY(double coefficient, Values::ShotParams a1, Values::ShotParams a2) {
  using ::frc971::shooter_interpolation::Blend;
  return Values::ShotParams{Blend(coefficient, a1.angle, a2.angle),
                    Blend(coefficient, a1.power, a2.power),
                    Blend(coefficient, a1.indexer_velocity, a2.indexer_velocity)};
}

}  // namespace constants
}  // namespace y2017
