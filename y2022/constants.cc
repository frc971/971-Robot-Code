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
#include "y2022/control_loops/superstructure/catapult/integral_catapult_plant.h"
#include "y2022/control_loops/superstructure/climber/integral_climber_plant.h"
#include "y2022/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2022/control_loops/superstructure/turret/integral_turret_plant.h"

namespace y2022 {
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

  // TODO(Yash): Set constants
  // Intake constants.
  auto *const intake_front = &r.intake_front;
  auto *const intake_back = &r.intake_back;

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

  intake_front->subsystem_params = intake_params;
  intake_back->subsystem_params = intake_params;

  // TODO(Yash): Set constants
  // Turret constants.
  auto *const turret = &r.turret;
  auto *const turret_params = &turret->subsystem_params;

  turret_params->zeroing_voltage = 4.0;
  turret_params->operating_voltage = 12.0;
  turret_params->zeroing_profile_params = {0.5, 2.0};
  turret_params->default_profile_params = {15.0, 40.0};
  turret_params->range = Values::kTurretRange();
  turret_params->make_integral_loop =
      control_loops::superstructure::turret::MakeIntegralTurretLoop;
  turret_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  turret_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kTurretEncoderRatio();
  turret_params->zeroing_constants.zeroing_threshold = 0.0005;
  turret_params->zeroing_constants.moving_buffer_size = 20;
  turret_params->zeroing_constants.allowable_encoder_error = 0.9;
  turret_params->zeroing_constants.measured_absolute_position = 0.0;

  // Climber constants
  auto *const climber = &r.climber;
  climber->subsystem_params.zeroing_voltage = 3.0;
  climber->subsystem_params.operating_voltage = 12.0;
  climber->subsystem_params.zeroing_profile_params = {0.5, 0.1};
  climber->subsystem_params.default_profile_params = {6.0, 1.0};
  climber->subsystem_params.range = Values::kClimberRange();
  climber->subsystem_params.make_integral_loop =
      control_loops::superstructure::climber::MakeIntegralClimberLoop;

  // Flipper arm constants
  Values::PotConstants flipper_arms;
  flipper_arms.subsystem_params.zeroing_voltage = 3.0;
  flipper_arms.subsystem_params.operating_voltage = 12.0;
  flipper_arms.subsystem_params.zeroing_profile_params = {0.5, 0.1};
  flipper_arms.subsystem_params.default_profile_params = {6.0, 1.0};
  flipper_arms.subsystem_params.range = Values::kFlipperArmRange();

  auto *const flipper_arm_right = &r.flipper_arm_right;
  auto *const flipper_arm_left = &r.flipper_arm_left;

  *flipper_arm_right = flipper_arms;
  *flipper_arm_left = flipper_arms;

  // No integral loops for flipper arms

  // Catapult
  Values::PotAndAbsEncoderConstants *const catapult = &r.catapult;
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      *const catapult_params = &catapult->subsystem_params;

  catapult_params->zeroing_voltage = 4.0;
  catapult_params->operating_voltage = 12.0;
  catapult_params->zeroing_profile_params = {0.5, 2.0};
  catapult_params->default_profile_params = {15.0, 40.0};
  catapult_params->range = Values::kCatapultRange();
  catapult_params->make_integral_loop =
      &control_loops::superstructure::catapult::MakeIntegralCatapultLoop;
  catapult_params->zeroing_constants.average_filter_size =
      Values::kZeroingSampleSize;
  catapult_params->zeroing_constants.one_revolution_distance =
      M_PI * 2.0 * constants::Values::kCatapultEncoderRatio();
  catapult_params->zeroing_constants.zeroing_threshold = 0.0005;
  catapult_params->zeroing_constants.moving_buffer_size = 20;
  catapult_params->zeroing_constants.allowable_encoder_error = 0.9;

  switch (team) {
    // A set of constants for tests.
    case 1:
      climber->potentiometer_offset = 0.0;
      intake_front->potentiometer_offset = 0.0;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      intake_back->potentiometer_offset = 0.0;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      turret->potentiometer_offset = 0.0;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      flipper_arm_left->potentiometer_offset = 0.0;
      flipper_arm_right->potentiometer_offset = 0.0;

      catapult_params->zeroing_constants.measured_absolute_position = 0.0;
      catapult->potentiometer_offset = 0.0;
      break;

    case kCompTeamNumber:
      climber->potentiometer_offset = 0.0;

      intake_front->potentiometer_offset = 2.79628370453323;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.248921954833972;

      intake_back->potentiometer_offset = 3.1409576474047;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.280099007470002;

      turret->potentiometer_offset =
          -9.99970387166721 + 0.06415943 + 0.073290115367682;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          0.511895084051468;

      flipper_arm_left->potentiometer_offset = -6.4;
      flipper_arm_right->potentiometer_offset = 5.66;

      catapult_params->zeroing_constants.measured_absolute_position =
          1.71723370408082;
      catapult->potentiometer_offset = -2.03383240293769;
      break;

    case kPracticeTeamNumber:
      climber->potentiometer_offset = 0.0;
      intake_front->potentiometer_offset = 0.0;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      intake_back->potentiometer_offset = 0.0;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      turret->potentiometer_offset = 0.0;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      flipper_arm_left->potentiometer_offset = 0.0;
      flipper_arm_right->potentiometer_offset = 0.0;

      catapult_params->zeroing_constants.measured_absolute_position = 0.0;
      catapult->potentiometer_offset = 0.0;
      break;

    case kCodingRobotTeamNumber:
      climber->potentiometer_offset = 0.0;
      intake_front->potentiometer_offset = 0.0;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      intake_back->potentiometer_offset = 0.0;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.0;
      turret->potentiometer_offset = 0.0;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          0.0;
      flipper_arm_left->potentiometer_offset = 0.0;
      flipper_arm_right->potentiometer_offset = 0.0;

      catapult_params->zeroing_constants.measured_absolute_position = 0.0;
      catapult->potentiometer_offset = 0.0;
      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2022
