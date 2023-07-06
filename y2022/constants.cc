#include "y2022/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "glog/logging.h"

#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "frc971/wpilib/wpilib_utils.h"
#include "y2022/control_loops/superstructure/catapult/integral_catapult_plant.h"
#include "y2022/control_loops/superstructure/climber/integral_climber_plant.h"
#include "y2022/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2022/control_loops/superstructure/turret/integral_turret_plant.h"

namespace y2022 {
namespace constants {

const int Values::kZeroingSampleSize;

Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;

  Values r;

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

  // Turret constants.
  auto *const turret = &r.turret;
  auto *const turret_params = &turret->subsystem_params;
  auto *turret_range = &r.turret_range;

  *turret_range = ::frc971::constants::Range{
      .lower_hard = -7.0,  // Back Hard
      .upper_hard = 3.4,   // Front Hard
      .lower = -6.5,       // Back Soft
      .upper = 3.15        // Front Soft
  };

  turret_params->zeroing_voltage = 4.0;
  turret_params->operating_voltage = 12.0;
  turret_params->zeroing_profile_params = {0.5, 2.0};
  turret_params->default_profile_params = {10.0, 20.0};
  turret_params->default_profile_params = {15.0, 20.0};
  turret_params->range = *turret_range;
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
  climber->subsystem_params.default_profile_params = {5.0, 1.0};
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

  // Interpolation table for comp and practice robots
  r.shot_interpolation_table = InterpolationTable<Values::ShotParams>({
      {1.0, {0.12, 19.4}},
      {1.6, {0.12, 19.4}},
      {1.9, {0.17, 19.4}},
      {2.12, {0.21, 19.4}},
      {2.9, {0.30, 19.9}},

      {3.2, {0.33, 20.1}},

      {3.60, {0.39, 20.65}},
      {4.50, {0.44, 22.3}},
      {4.9, {0.43, 22.75}},  // up to here
      {5.4, {0.43, 23.85}},

      {6.0, {0.42, 25.3}},
      {7.0, {0.40, 27.7}},

      {10.0, {0.40, 27.7}},
  });

  if (false) {
    // 1.5 meters -> 2.7
    // 2.3 meters -> 4.7
    // 4.5 meters -> 7.0
    // 7.0 meters -> 9.0

    constexpr double kShotVelocity = 9.0;
    r.shot_velocity_interpolation_table =
        InterpolationTable<Values::ShotVelocityParams>({
            {1.0, {kShotVelocity}},
            {10.0, {kShotVelocity}},
        });
  } else {
    r.shot_velocity_interpolation_table =
        InterpolationTable<Values::ShotVelocityParams>({
            {1.0, {2.7}},
            {1.5, {2.7}},
            {2.3, {4.7}},
            {4.5, {7.0}},
            {7.0, {9.0}},
            {10.0, {9.0}},
        });
  }

  Values::BallColorParams *const ball_color = &r.ball_color;

  switch (team) {
    // A set of constants for tests.
    case 1:
      r.shot_interpolation_table = InterpolationTable<Values::ShotParams>({
          {2, {0.08, 8.0}},
          {5, {0.6, 10.0}},
      });

      r.shot_velocity_interpolation_table =
          InterpolationTable<Values::ShotVelocityParams>({
              {2, {2.0}},
              {5, {4.0}},
          });

      climber->potentiometer_offset = -0.035;
      intake_front->potentiometer_offset = 3.122;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.175;
      intake_back->potentiometer_offset = 3.365;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.051;
      turret->potentiometer_offset = -7.932;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          1.166;
      flipper_arm_left->potentiometer_offset = -6.40;
      flipper_arm_right->potentiometer_offset = 5.56;

      catapult_params->zeroing_constants.measured_absolute_position = 0.0;
      catapult->potentiometer_offset = -2.033;

      ball_color->reference_red = {0, 0, 1, 1};
      ball_color->reference_blue = {0, 0, 1, 1};
      ball_color->ball_location = {0, 0, 1, 1};

      break;

    case kCompTeamNumber:
      climber->potentiometer_offset = -0.0463847608752 - 0.0376876182111 +
                                      0.0629263851579 - 0.00682128836400001 +
                                      0.0172237531191 - 0.0172237531191 +
                                      0.00443383743660001 - 0.0117667224279;

      intake_front->potentiometer_offset = +3.572389;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.238611243887673;

      intake_back->potentiometer_offset =
          3.1409576474047 + 0.278653334013286 + 0.00879137908308503 +
          0.0837134053818833 + 0.832945730100298 - 0.00759895654985426 -
          2.03114758819475 + 0.318379597392509 + 0.675664531140745 +
          0.0650864893911517 - 0.0202318432257168 - 0.0561212375592096;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.292329083185933;

      turret->potentiometer_offset =
          -9.99970387166721 + 0.06415943 + 0.073290115367682 -
          0.0634440443622909 + 0.213601224728352 + 0.0657973101027296 -
          0.114726411377978 - 0.980314029089968 - 0.0266013159299456 +
          0.0631240002215899 + 0.222882504808653 + 0.0370686419434252 -
          0.0965027214840068 - 0.126737479717192 - 0.0773753775457 +
          2.8132444751306;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          1.16683731504739;

      flipper_arm_left->potentiometer_offset = -6.4;
      flipper_arm_right->potentiometer_offset = 5.56;

      *turret_range = ::frc971::constants::Range{
          .lower_hard = -7.0,  // Back Hard
          .upper_hard = 3.4,   // Front Hard
          .lower = -6.4,       // Back Soft
          .upper = 2.9         // Front Soft
      };
      turret_params->range = *turret_range;

      catapult_params->zeroing_constants.measured_absolute_position =
          1.71723370408082;
      catapult->potentiometer_offset = -2.03383240293769;

      ball_color->reference_red = {440, 150, 50, 130};
      ball_color->reference_blue = {440, 350, 30, 100};
      ball_color->ball_location = {100, 400, 140, 50};

      break;

    case kPracticeTeamNumber:
      catapult_params->range.lower = -0.885;

      r.shot_interpolation_table = InterpolationTable<Values::ShotParams>({
          {1.0, {0.08, 20.0}},
          {1.6, {0.08, 20.0}},
          {1.9, {0.11, 20.0}},
          {2.12, {0.15, 20.5}},
          {2.9, {0.27, 20.2}},

          {3.2, {0.29, 20.6}},

          {3.60, {0.36, 21.0}},
          {4.50, {0.41, 22.7}},
          {4.9, {0.42, 23.3}},
          {5.4, {0.42, 24.6}},

          {6.0, {0.42, 26.25}},
          {7.0, {0.39, 28.25}},

          {10.0, {0.39, 28.25}},
      });

      climber->potentiometer_offset =
          -0.1209073362519 + 0.0760598 - 0.0221716219244 - 0.00321684;
      intake_front->potentiometer_offset = 3.06604378582351 - 0.60745632979918;
      intake_front->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.143667561169188;
      intake_back->potentiometer_offset =
          3.10861174832838 + 0.431432052414186 - 0.171422335492571 +
          0.0414174770317617 + 0.0908883523752557 - 0.0632767010207473;
      intake_back->subsystem_params.zeroing_constants
          .measured_absolute_position = 0.404628372743507;

      turret->potentiometer_offset =
          -8.14418207451834 + 0.342635491808218 - 0.944807955598189 -
          0.0718028442723373 - 0.0793332946417493 + 0.233707527214682 +
          0.0828349540635251 + 0.677740533247017 - 0.0828349540635251 -
          0.0903654044329345 - 0.105426305171759 - 0.150609007388226 -
          0.0338870266623506 - 0.0677740533247011 - 0.135548106649404 - 0.6852;
      turret->subsystem_params.zeroing_constants.measured_absolute_position =
          0.8306;
      *turret_range = ::frc971::constants::Range{
          .lower_hard = -7.0,  // Back Hard
          .upper_hard = 3.4,   // Front Hard
          .lower = -6.4,       // Back Soft
          .upper = 2.9         // Front Soft
      };
      turret_params->range = *turret_range;
      flipper_arm_left->potentiometer_offset =
          -4.39536583413615 - 0.108401297910291;
      flipper_arm_right->potentiometer_offset =
          4.36264091401229 + 0.175896445665755;

      catapult_params->zeroing_constants.measured_absolute_position =
          1.62909518684227;
      catapult->potentiometer_offset = -1.52951814169821 - 0.0200812009850977;

      ball_color->reference_red = {526, 75, 110, 220};
      ball_color->reference_blue = {526, 340, 110, 100};
      ball_color->ball_location = {40, 440, 200, 30};

      break;

    case kCodingRobotTeamNumber:
      r.shot_interpolation_table = InterpolationTable<Values::ShotParams>({
          {2, {0.08, 8.0}},
          {5, {0.6, 10.0}},
      });

      r.shot_velocity_interpolation_table =
          InterpolationTable<Values::ShotVelocityParams>({
              {2, {2.0}},
              {5, {4.0}},
          });

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

      ball_color->reference_red = {0, 0, 1, 1};
      ball_color->reference_blue = {0, 0, 1, 1};
      ball_color->ball_location = {0, 0, 1, 1};

      break;

    default:
      LOG(FATAL) << "unknown team: " << team;
  }

  CHECK(frc971::wpilib::SafePotVoltageRange(
      Values::kClimberRange(), climber->potentiometer_offset,
      [](double meters) { return meters / Values::kClimberPotMetersPerVolt(); },
      false))
      << "Couldn't translate climber pot";
  CHECK(frc971::wpilib::SafePotVoltageRange(
      Values::kFlipperArmRange(), flipper_arm_left->potentiometer_offset,
      [](double radians) {
        return radians / Values::kFlipperArmsPotRadiansPerVolt();
      },
      false))
      << "Couldn't translate flipper left pot";
  CHECK(frc971::wpilib::SafePotVoltageRange(
      Values::kFlipperArmRange(), flipper_arm_right->potentiometer_offset,
      [](double radians) {
        return radians / Values::kFlipperArmsPotRadiansPerVolt();
      },
      true))
      << "Couldn't translate flipper right pot";
  CHECK(frc971::wpilib::SafePotVoltageRange(
      Values::kIntakeRange(), intake_front->potentiometer_offset,
      [](double radians) {
        return radians / Values::kIntakePotRadiansPerVolt();
      },
      true))
      << "Couldn't translate front intake pot";
  CHECK(frc971::wpilib::SafePotVoltageRange(
      Values::kIntakeRange(), intake_back->potentiometer_offset,
      [](double radians) {
        return radians / Values::kIntakePotRadiansPerVolt();
      },
      true))
      << "Couldn't translate back intake pot";
  CHECK(frc971::wpilib::SafePotVoltageRange(
      *turret_range, turret->potentiometer_offset,
      [](double radians) {
        return radians / Values::kTurretPotRadiansPerVolt();
      },
      false))
      << "Couldn't translate turret pot";

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }

}  // namespace constants
}  // namespace y2022
