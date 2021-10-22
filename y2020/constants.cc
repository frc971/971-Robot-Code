#include "y2020/constants.h"

#include <cinttypes>
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

const Values *DoGetValuesForTeam(uint16_t team) {
  Values *const r = new Values();
  ::frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
      ::frc971::zeroing::AbsoluteAndAbsoluteEncoderZeroingEstimator>
      *const hood = &r->hood;
  Values::HoodGeometry *const hood_geometry = &r->hood_geometry;

  // We found that the finisher velocity does not change ball velocity much,
  // so keep it constant.
  constexpr double kVelocityFinisher = 350.0;
  r->shot_interpolation_table =
      InterpolationTable<Values::ShotParams>({{1.0, {0.01, 10.7}},
                                              {1.2, {0.01, 10.7}},
                                              {1.4732, {0.10, 10.6}},
                                              {2.5, {0.36, 12.0}},
                                              {3.50, {0.43, 13.2}},
                                              {4.7371, {0.535, 14.2}},
                                              {5.27, {0.53, 14.55}},
                                              {6.332, {0.53, 15.2}},
                                              {7.48, {0.55, 17.0}},
                                              {8.30, {0.565, 17.0}},
                                              {9.20, {0.535, 17.0}}});

  r->flywheel_shot_interpolation_table =
      InterpolationTable<Values::FlywheelShotParams>(
          {{10.6, {250.0, kVelocityFinisher}},
           {12.0, {275.0, kVelocityFinisher}},
           {13.2, {300.0, kVelocityFinisher}},
           {14.0, {325.0, kVelocityFinisher}},
           {14.6, {350.0, kVelocityFinisher}},
           {15.2, {375.0, kVelocityFinisher}},
           {15.6, {400.0, kVelocityFinisher}},
           {16.1, {425.0, kVelocityFinisher}},
           {16.3, {450.0, kVelocityFinisher}},
           {16.6, {475.0, kVelocityFinisher}},
           {17.0, {500.0, kVelocityFinisher}}});

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

  constexpr double kDegToRad = M_PI / 180.0;
  constexpr double kMmToM = 1.0 / 1000.0;
  hood_geometry->theta_0 = 22.98004 * kDegToRad;
  hood_geometry->screw_length_0 = 110.33888 * kMmToM;
  hood_geometry->radius = 269.6262 * kMmToM;
  hood_geometry->diagonal_length = 288.4353 * kMmToM;
  hood_geometry->back_plate_diagonal_length = 22.86 * kMmToM;

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
    case Values::kSpareRoborioTeamNumber:
      break;

    case Values::kCompTeamNumber:
      intake->zeroing_constants.measured_absolute_position =
          0.433936997731885 - Values::kIntakeZero();

      turret->potentiometer_offset = 5.52519370141247 + 0.00853506822980376 +
                                     0.0109413725126625 - 0.223719825811759 +
                                     0.261356551915472 - 0.0490168170767848 -
                                     0.179342788816305;
      turret_params->zeroing_constants.measured_absolute_position =
          2.75051496009509;

      hood->zeroing_constants.measured_absolute_position = 0.0344482433884915;
      hood->zeroing_constants.single_turn_measured_absolute_position =
          0.31055891442198;
      break;

    case Values::kPracticeTeamNumber:
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

const Values *values = nullptr;

void DoGetValues() {
  uint16_t team = ::aos::network::GetTeamNumber();
  AOS_LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
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
}  // namespace y2020
