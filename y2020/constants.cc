#include "y2020/constants.h"

#include <inttypes.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "y2020/control_loops/superstructure/control_panel/integral_control_panel_plant.h"
#include "y2020/control_loops/superstructure/hood/integral_hood_plant.h"
#include "y2020/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2020/control_loops/superstructure/turret/integral_turret_plant.h"

namespace y2020 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kSpareRoborioTeamNumber = 6971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator>
      *const hood = &r->hood;

  constexpr double kFeetToMeters = 0.0254 * 12.0;
  // Approximate robot length, for converting estimates from the doc below.
  // Rounded up from exact estimate, since I'm not sure if the original estimate
  // includes bumpers.
  constexpr double kRobotLength = 0.9;
  // { distance_to_target, { hood_angle, accelerator_power, finisher_power }}
  // Current settings based on
  // https://docs.google.com/document/d/1NR9F-ntlSoqZ9LqDzLjn-c14t8ZrppawCCG7wQy47RU/edit
  r->shot_interpolation_table = InterpolationTable<Values::ShotParams>(
      {{7.6 * kFeetToMeters - kRobotLength, {0.115, 197.0, 175.0}},
       {7.6 * kFeetToMeters + kRobotLength, {0.31, 265.0, 235.0}},
       {12.6 * kFeetToMeters + kRobotLength, {0.4, 292.0, 260.0}},
       {17.6 * kFeetToMeters + kRobotLength, {0.52, 365.0, 325.0}}});

  // Hood constants.
  hood->zeroing_voltage = 2.0;
  hood->operating_voltage = 12.0;
  hood->zeroing_profile_params = {0.5, 3.0};
  hood->default_profile_params = {6.0, 30.0};
  hood->range = Values::kHoodRange();
  hood->make_integral_loop =
      control_loops::superstructure::hood::MakeIntegralHoodLoop;
  hood->zeroing_constants.average_filter_size = Values::kZeroingSampleSize;
  hood->zeroing_constants.zeroing_threshold = 0.0005;
  hood->zeroing_constants.moving_buffer_size = 20;
  hood->zeroing_constants.allowable_encoder_error = 0.9;
  hood->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kHoodEncoderRatio();
  hood->zeroing_constants.single_turn_middle_position =
      Values::kHoodRange().middle();
  hood->zeroing_constants.single_turn_one_revolution_distance =
      M_PI * 2.0 * constants::Values::kHoodSingleTurnEncoderRatio();
  hood->zeroing_constants.measured_absolute_position = 0;
  hood->zeroing_constants.single_turn_measured_absolute_position = 0;

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator> *const intake =
      &r->intake;

  // Intake constants.
  intake->zeroing_voltage = 3.0;
  intake->operating_voltage = 12.0;
  intake->zeroing_profile_params = {0.5, 3.0};
  intake->default_profile_params = {6.0, 30.0};
  intake->range = Values::kIntakeRange();
  intake->make_integral_loop =
      control_loops::superstructure::intake::MakeIntegralIntakeLoop;
  intake->zeroing_constants.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeEncoderRatio();
  intake->zeroing_constants.zeroing_threshold = 0.0005;
  intake->zeroing_constants.moving_buffer_size = 20;
  intake->zeroing_constants.allowable_encoder_error = 0.9;
  intake->zeroing_constants.middle_position = Values::kIntakeRange().middle();

  Values::PotAndAbsEncoderConstants *const turret = &r->turret;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const turret_params = &turret->subsystem_params;

  // Turret Constants
  turret_params->zeroing_voltage = 4.0;
  turret_params->operating_voltage = 8.0;
  // TODO(austin): Tune these.
  turret_params->zeroing_profile_params = {0.5, 2.0};
  turret_params->default_profile_params = {15.0, 40.0};
  turret_params->range = Values::kTurretRange();
  turret_params->make_integral_loop =
      &control_loops::superstructure::turret::MakeIntegralTurretLoop;
  turret_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  turret_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kTurretEncoderRatio();
  turret_params->zeroing_constants.zeroing_threshold = 0.0005;
  turret_params->zeroing_constants.moving_buffer_size = 20;
  turret_params->zeroing_constants.allowable_encoder_error = 0.9;

  CHECK_LE(hood->range.range(),
           hood->zeroing_constants.single_turn_one_revolution_distance);
  CHECK_LE(intake->range.range(),
           intake->zeroing_constants.one_revolution_distance);

  switch (team) {
    // A set of constants for tests.
    case 1:
    case kSpareRoborioTeamNumber:
      break;

    case kCompTeamNumber:
      intake->zeroing_constants.measured_absolute_position =
          1.42977866919024 - Values::kIntakeZero();

      turret->potentiometer_offset = 5.52519370141247 + 0.00853506822980376 +
                                     0.0109413725126625 - 0.223719825811759;
      turret_params->zeroing_constants.measured_absolute_position =
          0.547478339799516;

      hood->zeroing_constants.measured_absolute_position = 0.0344482433884915;
      hood->zeroing_constants.single_turn_measured_absolute_position =
          0.31055891442198;
      break;

    case kPracticeTeamNumber:
      hood->zeroing_constants.measured_absolute_position = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.347;

      turret->potentiometer_offset = 5.3931926228241;
      turret_params->zeroing_constants.measured_absolute_position = 4.22;
      break;

    case Values::kCodingRobotTeamNumber:
      hood->zeroing_constants.measured_absolute_position = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;

      turret->potentiometer_offset = 0.0;
      turret_params->zeroing_constants.measured_absolute_position = 0.0;
      break;

    default:
      AOS_LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

void DoGetValues(const Values **result) {
  uint16_t team = ::aos::network::GetTeamNumber();
  AOS_LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  *result = DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static absl::once_flag once;
  static const Values *result;
  absl::call_once(once, DoGetValues, &result);
  return *result;
}

const Values &GetValuesForTeam(uint16_t team_number) {
  static aos::stl_mutex mutex;
  std::unique_lock<aos::stl_mutex> locker(mutex);

  // IMPORTANT: This declaration has to stay after the mutex is locked to
  // avoid race conditions.
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
}  // namespace y2020
