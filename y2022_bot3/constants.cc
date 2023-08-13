#include "y2022_bot3/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "glog/logging.h"

#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "frc971/zeroing/pot_and_absolute_encoder.h"
#include "y2022_bot3/control_loops/superstructure/climber/integral_climber_plant.h"
#include "y2022_bot3/control_loops/superstructure/intake/integral_intake_plant.h"

namespace y2022_bot3 {
namespace constants {

const int Values::kZeroingSampleSize;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

}  // namespace

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;

  auto *const intake = &r.intake;
  auto *const climber_left = &r.climber_left;
  auto *const climber_right = &r.climber_right;

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      intake_params;

  intake_params.zeroing_voltage = 3.0;
  intake_params.operating_voltage = 12.0;
  intake_params.zeroing_profile_params = {0.5, 3.0};
  intake_params.default_profile_params = {6.0, 30.0};
  intake_params.range = Values::kIntakeRange();
  intake_params.make_integral_loop =
      control_loops::superstructure::intake::MakeIntegralIntakeLoop;
  intake_params.zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  intake_params.zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kIntakeEncoderRatio();
  intake_params.zeroing_constants.zeroing_threshold = 0.0005;
  intake_params.zeroing_constants.moving_buffer_size = 20;
  intake_params.zeroing_constants.allowable_encoder_error = 0.9;
  intake_params.zeroing_constants.measured_absolute_position = 0.0;

  intake->subsystem_params = intake_params;

  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      climber_params;

  climber_params.zeroing_voltage = 3.0;
  climber_params.operating_voltage = 12.0;
  climber_params.zeroing_profile_params = {0.5, 0.1};
  climber_params.default_profile_params = {5.0, 1.0};
  climber_params.range = Values::kClimberRange();
  climber_params.make_integral_loop =
      control_loops::superstructure::climber::MakeIntegralClimberLoop;
  climber_params.zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  climber_params.zeroing_constants.one_revolution_distance =
      constants::Values::kClimberEncoderRatio() *
      constants::Values::kClimberEncoderMetersPerRevolution();
  climber_params.zeroing_constants.zeroing_threshold = 0.0005;
  climber_params.zeroing_constants.moving_buffer_size = 20;
  climber_params.zeroing_constants.allowable_encoder_error = 0.9;
  climber_params.zeroing_constants.measured_absolute_position = 0.0;

  climber_left->subsystem_params = climber_params;
  climber_right->subsystem_params = climber_params;

  switch (team) {
    // A set of constants for tests.
    case 1:
      intake->potentiometer_offset = 0.0;
      intake->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      climber_left->potentiometer_offset = 0.0;
      climber_left->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      climber_right->potentiometer_offset = 0.0;
      climber_right->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      break;

    case kCompTeamNumber:
      intake->potentiometer_offset = 0.0;
      intake->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      climber_left->potentiometer_offset = 0.0;
      climber_left->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      climber_right->potentiometer_offset = 0.0;
      climber_right->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      break;

    case kPracticeTeamNumber:
      intake->potentiometer_offset = 0.0;
      intake->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      climber_left->potentiometer_offset = 0.0;
      climber_left->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      climber_right->potentiometer_offset = 0.0;
      climber_right->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      break;

    case kCodingRobotTeamNumber:
      intake->potentiometer_offset = 0.0;
      intake->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      climber_left->potentiometer_offset = 0.0;
      climber_left->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      climber_right->potentiometer_offset = 0.0;
      climber_right->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2022_bot3
