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
#include "y2019/control_loops/superstructure/elevator/integral_elevator_plant.h"
#include "y2019/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2019/control_loops/superstructure/stilts/integral_stilts_plant.h"
#include "y2019/control_loops/superstructure/wrist/integral_wrist_plant.h"

namespace y2019 {
namespace constants {

using ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator;

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  Values::PotAndAbsConstants *const elevator = &r->elevator;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const elevator_params = &(elevator->subsystem_params);
  Values::PotAndAbsConstants *const stilts = &r->stilts;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const stilts_params = &(stilts->subsystem_params);
  Values::PotAndAbsConstants *const wrist = &r->wrist;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const wrist_params = &(wrist->subsystem_params);
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteEncoderZeroingEstimator> *const intake =
      &r->intake;

  // Elevator constants.
  elevator_params->zeroing_voltage = 4.0;
  elevator_params->operating_voltage = 12.0;
  elevator_params->zeroing_profile_params = {0.1, 1.0};
  elevator_params->default_profile_params = {4.0, 3.0};
  elevator_params->range = Values::kElevatorRange();
  elevator_params->make_integral_loop =
      &control_loops::superstructure::elevator::MakeIntegralElevatorLoop;
  elevator_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  elevator_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kElevatorEncoderRatio();
  elevator_params->zeroing_constants.zeroing_threshold = 0.005;
  elevator_params->zeroing_constants.moving_buffer_size = 20;
  elevator_params->zeroing_constants.allowable_encoder_error = 0.9;

  // Wrist constants.
  wrist_params->zeroing_voltage = 4.0;
  wrist_params->operating_voltage = 12.0;
  wrist_params->zeroing_profile_params = {0.5, 2.0};
  wrist_params->default_profile_params = {6.0, 5.0};
  wrist_params->range = Values::kWristRange();
  wrist_params->make_integral_loop =
      &control_loops::superstructure::wrist::MakeIntegralWristLoop;
  wrist_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  wrist_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kWristEncoderRatio();
  wrist_params->zeroing_constants.zeroing_threshold = 0.0005;
  wrist_params->zeroing_constants.moving_buffer_size = 20;
  wrist_params->zeroing_constants.allowable_encoder_error = 0.9;

  // Intake constants.
  intake->zeroing_voltage = 4.0;
  intake->operating_voltage = 12.0;
  intake->zeroing_profile_params = {0.5, 3.0};
  intake->default_profile_params = {6.0, 5.0};
  intake->range = Values::kIntakeRange();
  intake->make_integral_loop =
      control_loops::superstructure::intake::MakeIntegralIntakeLoop;
  intake->zeroing_constants.average_filter_size = Values::kZeroingSampleSize;
  intake->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeEncoderRatio();
  intake->zeroing_constants.zeroing_threshold = 0.0005;
  intake->zeroing_constants.moving_buffer_size = 20;
  intake->zeroing_constants.allowable_encoder_error = 0.9;

  // Stilts constants.
  stilts_params->zeroing_voltage = 4.0;
  stilts_params->operating_voltage = 12.0;
  stilts_params->zeroing_profile_params = {0.1, 3.0};
  stilts_params->default_profile_params = {2.0, 4.0};
  stilts_params->range = Values::kStiltsRange();
  stilts_params->make_integral_loop =
      &control_loops::superstructure::stilts::MakeIntegralStiltsLoop;
  stilts_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  stilts_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kStiltsEncoderRatio();
  stilts_params->zeroing_constants.zeroing_threshold = 0.0005;
  stilts_params->zeroing_constants.moving_buffer_size = 20;
  stilts_params->zeroing_constants.allowable_encoder_error = 0.9;

  switch (team) {
    // A set of constants for tests.
    case 1:
      elevator_params->zeroing_constants.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;
      intake->zeroing_constants.middle_position = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;
      break;

    case kCompTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;
      intake->zeroing_constants.middle_position = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;
      break;

    case kPracticeTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.049419;
      elevator->potentiometer_offset = -0.022320;

      intake->zeroing_constants.measured_absolute_position = 2.303729;
      intake->zeroing_constants.middle_position =
          Values::kIntakeRange().middle();

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.357394;
      wrist->potentiometer_offset = -1.479097 - 2.740303;

      stilts_params->zeroing_constants.measured_absolute_position = 0.047838;
      stilts->potentiometer_offset = -0.093820;
      break;

    case kCodingRobotTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;
      intake->zeroing_constants.middle_position = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;
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
