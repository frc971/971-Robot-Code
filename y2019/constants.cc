#include "y2019/constants.h"

#include <inttypes.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "aos/once.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2019 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  Values::PotAndAbsConstants *const elevator = &r->elevator;
  Values::Intake *const intake = &r->intake;
  Values::PotAndAbsConstants *const stilts = &r->stilts;
  Values::PotAndAbsConstants *const wrist = &r->wrist;

  elevator->zeroing.average_filter_size = Values::kZeroingSampleSize;
  elevator->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kElevatorEncoderRatio();
  elevator->zeroing.zeroing_threshold = 0.0005;
  elevator->zeroing.moving_buffer_size = 20;
  elevator->zeroing.allowable_encoder_error = 0.9;

  intake->zeroing.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeEncoderRatio();
  intake->zeroing.zeroing_threshold = 0.0005;
  intake->zeroing.moving_buffer_size = 20;
  intake->zeroing.allowable_encoder_error = 0.9;

  stilts->zeroing.average_filter_size = Values::kZeroingSampleSize;
  stilts->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kStiltsEncoderRatio();
  stilts->zeroing.zeroing_threshold = 0.0005;
  stilts->zeroing.moving_buffer_size = 20;
  stilts->zeroing.allowable_encoder_error = 0.9;

  wrist->zeroing.average_filter_size = Values::kZeroingSampleSize;
  wrist->zeroing.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kWristEncoderRatio();
  wrist->zeroing.zeroing_threshold = 0.0005;
  wrist->zeroing.moving_buffer_size = 20;
  wrist->zeroing.allowable_encoder_error = 0.9;

  switch (team) {
    // A set of constants for tests.
    case 1:
      elevator->zeroing.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing.measured_absolute_position = 0.0;
      intake->zeroing.middle_position = 0.0;

      stilts->zeroing.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      wrist->zeroing.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;
      break;

    case kCompTeamNumber:
      elevator->zeroing.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing.measured_absolute_position = 0.0;
      intake->zeroing.middle_position = 0.0;

      stilts->zeroing.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      wrist->zeroing.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;
      break;

    case kPracticeTeamNumber:
      elevator->zeroing.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing.measured_absolute_position = 0.0;
      intake->zeroing.middle_position = 0.0;

      stilts->zeroing.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      wrist->zeroing.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;
      break;

    case kCodingRobotTeamNumber:
      elevator->zeroing.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing.measured_absolute_position = 0.0;
      intake->zeroing.middle_position = 0.0;

      stilts->zeroing.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      wrist->zeroing.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;
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
}  // namespace y2019
