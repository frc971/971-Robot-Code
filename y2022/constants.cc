#include "y2022/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "glog/logging.h"
#include "y2022/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2022/control_loops/superstructure/climber/integral_climber_plant.h"

namespace y2022 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();

  // TODO(Yash): Set constants
  // Intake constants.
  auto *const intake = &r->intake;

  intake->zeroing_voltage = 3.0;
  intake->operating_voltage = 12.0;
  intake->zeroing_profile_params = {0.5, 3.0};
  intake->default_profile_params = {6.0, 30.0};
  intake->range = Values::kIntakeRange();
  intake->make_integral_loop =
      control_loops::superstructure::intake::MakeIntegralIntakeLoop;

  // The number of samples in the moving average filter.
  intake->zeroing_constants.average_filter_size = Values::kZeroingSampleSize;
  // The distance that the absolute encoder needs to complete a full rotation.
  intake->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeEncoderRatio();

  // Threshold for deciding if we are moving. moving_buffer_size samples need to
  // be within this distance of each other before we use the middle one to zero.
  intake->zeroing_constants.zeroing_threshold = 0.0005;
  // Buffer size for deciding if we are moving.
  intake->zeroing_constants.moving_buffer_size = 20;

  // Value between 0 and 1 indicating what fraction of one_revolution_distance
  // it is acceptable for the offset to move.
  intake->zeroing_constants.allowable_encoder_error = 0.9;

  // Measured absolute position of the encoder when at zero.
  intake->zeroing_constants.measured_absolute_position = 0.0;

  // Climber constants
  auto *const climber = &r->climber;
  climber->subsystem_params.zeroing_voltage = 3.0;
  climber->subsystem_params.operating_voltage = 12.0;
  climber->subsystem_params.zeroing_profile_params = {0.5, 0.1};
  climber->subsystem_params.default_profile_params = {6.0, 1.0};
  climber->subsystem_params.range = Values::kClimberRange();
  climber->subsystem_params.make_integral_loop =
      control_loops::superstructure::climber::MakeIntegralClimberLoop;

  switch (team) {
    // A set of constants for tests.
    case 1:
      climber->potentiometer_offset = 0.0;
      break;

    case kCompTeamNumber:
      climber->potentiometer_offset = 0.0;
      break;

    case kPracticeTeamNumber:
      climber->potentiometer_offset = 0.0;
      break;

    case kCodingRobotTeamNumber:
      climber->potentiometer_offset = 0.0;
      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  return r;
}

const Values *values = nullptr;

void DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO) << "creating a Constants for team: " << team;
  values = DoGetValuesForTeam(team);
}

}  // namespace

void InitValues() {
  static absl::once_flag once;
  absl::call_once(once, DoGetValues);
}

const Values &GetValues() {
  CHECK(values)
      << "Values are uninitialized. Call InitValues before accessing them.";
  return *values;
}

}  // namespace constants
}  // namespace y2022
