#include "y2020/constants.h"

#include <cinttypes>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "aos/network/team_number.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "glog/logging.h"
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
  r->shot_interpolation_table =
      InterpolationTable<Values::ShotParams>({{1.128, {0.01, 225, 170}},
                                              {1.509, {0.01, 215, 160}},
                                              {1.89, {0.01, 215, 155}},
                                              {2.15, {0.093, 210, 170}},
                                              {2.68, {0.195, 240, 190}},
                                              {3.19, {0.27, 250, 220}},
                                              {3.93, {0.365, 285, 250}},
                                              {4.63, {0.42, 320, 280}},
                                              {5.32, {0.515, 375, 330}},
                                              {6, {0.565, 440, 400}},
                                              {6.68, {0.58, 480, 450}},
                                              {7.37, {0.645, 520, 540}},
                                              {8.36, {0.645, 550, 560}},
                                              {9.39, {0.66, 550, 580}},
                                              {10.4, {0.67, 550, 600}}});

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
      r->shot_interpolation_table = InterpolationTable<Values::ShotParams>(
          {{1, {0.01, 250, 250}}, {10, {0.67, 500, 600}}});
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

      hood->zeroing_constants.measured_absolute_position = 0.0102882878327271;
      hood->zeroing_constants.single_turn_measured_absolute_position =
          0.302574797776192;
      break;

    case Values::kPracticeTeamNumber:
      hood->zeroing_constants.measured_absolute_position = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.205469223604347;

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
}  // namespace y2020
