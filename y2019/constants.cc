#include "y2019/constants.h"

#include <inttypes.h>

#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"
#include "aos/network/team_number.h"
#include "absl/base/call_once.h"
#include "y2019/control_loops/superstructure/elevator/integral_elevator_plant.h"
#include "y2019/control_loops/superstructure/intake/integral_intake_plant.h"
#include "y2019/control_loops/superstructure/stilts/integral_stilts_plant.h"
#include "y2019/control_loops/superstructure/wrist/integral_wrist_plant.h"
#include "y2019/vision/constants.h"

namespace y2019 {
namespace constants {

using ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator;

const int Values::kZeroingSampleSize;
constexpr size_t Values::kNumCameras;

namespace {

const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;
const uint16_t kCodingRobotTeamNumber = 7971;

constexpr double FeetToMeters(double ft) { return 0.3048 * ft; }
constexpr double InchToMeters(double in) { return 0.0254 * in; }
constexpr double DegToRad(double deg) { return deg * M_PI / 180.0; }

// Populates camera Pose information from the calibrated vision constants.
void FillCameraPoses(
    uint32_t teensy_processor_id,
    ::std::array<Values::CameraCalibration, Values::kNumCameras> *cameras) {
  ::std::array<int, 5> camera_ids =
      vision::CameraSerialNumbers(teensy_processor_id);
  for (size_t ii = 0; ii < camera_ids.size(); ++ii) {
    const vision::CameraCalibration *calibration =
        vision::GetCamera(camera_ids[ii]);
    if (calibration != nullptr) {
      vision::CameraGeometry geometry = calibration->geometry;
      *cameras->at(ii).pose.mutable_pos() << geometry.location[0],
          geometry.location[1], geometry.location[2];
      cameras->at(ii).pose.set_theta(geometry.heading);
    }
  }
}

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
  elevator_params->zeroing_voltage = 3.0;
  elevator_params->operating_voltage = 12.0;
  elevator_params->zeroing_profile_params = {0.1, 1.0};
  elevator_params->default_profile_params = {4.0, 13.0};
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
  wrist_params->default_profile_params = {10.0, 40.0};
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

  // Stilts constants.
  stilts_params->zeroing_voltage = 3.0;
  stilts_params->operating_voltage = 12.0;
  stilts_params->zeroing_profile_params = {0.1, 0.5};
  stilts_params->default_profile_params = {0.15, 0.5};
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

  r->camera_noise_parameters = {.max_viewable_distance = 10.0,
                                .heading_noise = 0.1,
                                .nominal_distance_noise = 0.15,
                                .nominal_skew_noise = 0.75,
                                .nominal_height_noise = 0.01};

  // Deliberately make FOV a bit large so that we are overly conservative in
  // our EKF.
  for (auto &camera : r->cameras) {
    camera.fov = M_PI_2 * 1.5;
  }

  switch (team) {
    // A set of constants for tests.
    case 1:
      elevator_params->zeroing_constants.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      // For the simulation, just place cameras at the center of the robot with
      // nominal angles (back/sides + 15 degree offset front cameras).
      r->cameras[0].pose.set_theta(M_PI);
      r->cameras[1].pose.set_theta(0.26);
      r->cameras[2].pose.set_theta(-0.26);
      r->cameras[3].pose.set_theta(M_PI_2);
      r->cameras[4].pose.set_theta(-M_PI_2);
      break;

    case kCompTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.145498;
      elevator->potentiometer_offset =
          -0.075017 + 0.015837 + 0.009793 - 0.012017 + 0.019915 + 0.010126 +
          0.005541 + 0.006088 - 0.039687 + 0.005843 + 0.009007 + 0.008604 -
          0.004621 + 0.003305;

      intake->zeroing_constants.measured_absolute_position = 1.273143;

      wrist_params->zeroing_constants.measured_absolute_position = 0.155868;
      wrist->potentiometer_offset = -4.257454 - 0.058039 + 0.270233 - 0.661464;

      stilts_params->zeroing_constants.measured_absolute_position = 0.066843;
      stilts->potentiometer_offset = -0.015760 + 0.011604 - 0.061213 + 0.006690;
      FillCameraPoses(vision::CompBotTeensyId(), &r->cameras);
      break;

    case kPracticeTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.131568;
      elevator->potentiometer_offset = -0.022320 + 0.020567 - 0.022355 -
                                       0.006497 + 0.019690 + 0.009151 -
                                       0.007513 + 0.007311;

      intake->zeroing_constants.measured_absolute_position =
          1.928755 + 0.205352;

      wrist_params->zeroing_constants.measured_absolute_position = 0.180039;
      wrist->potentiometer_offset = -4.200894 - 0.187134;

      stilts_params->zeroing_constants.measured_absolute_position = 0.050556;
      stilts->potentiometer_offset =
          -0.093820 + 0.0124 - 0.008334 + 0.004507 - 0.007973 + -0.001221;

      FillCameraPoses(vision::PracticeBotTeensyId(), &r->cameras);
      break;

    case kCodingRobotTeamNumber:
      elevator_params->zeroing_constants.measured_absolute_position = 0.0;
      elevator->potentiometer_offset = 0.0;

      intake->zeroing_constants.measured_absolute_position = 0.0;

      wrist_params->zeroing_constants.measured_absolute_position = 0.0;
      wrist->potentiometer_offset = 0.0;

      stilts_params->zeroing_constants.measured_absolute_position = 0.0;
      stilts->potentiometer_offset = 0.0;

      FillCameraPoses(vision::CodeBotTeensyId(), &r->cameras);

      break;

    default:
      AOS_LOG(FATAL, "unknown team #%" PRIu16 "\n", team);
  }

  return r;
}

void DoGetValues(const Values** result) {
  uint16_t team = ::aos::network::GetTeamNumber();
  AOS_LOG(INFO, "creating a Constants for team %" PRIu16 "\n", team);
  *result = DoGetValuesForTeam(team);
}

}  // namespace

const Values &GetValues() {
  static absl::once_flag once;
  static const Values* result;
  absl::call_once(once, DoGetValues, &result);
  return *result;
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

constexpr size_t Field::kNumTargets;
constexpr size_t Field::kNumObstacles;

Field::Field() {
  // TODO(james): These values need to re-verified. I got them by skimming the
  // manual and they all seem to be pretty much correct.
  //
  // Note: Per //frc971/control_loops/pose.h, coordinate system is:
  // -In meters
  // -Origin at center of our driver's station wall
  // -Positive X-axis pointing straight out from driver's station
  // -Positive Y-axis pointing straight left from the driver's perspective
  // -Positive Z-axis is straight up.
  // -The angle of the target is such that the angle is the angle you would
  //  need to be facing to see it straight-on. I.e., if the target angle is
  //  pi / 2.0, then you would be able to see it face on by facing straight
  //  left from the driver's point of view (if you were standing in the right
  //  spot).
  constexpr double kCenterFieldX = FeetToMeters(27.0) + InchToMeters(1.125);

  constexpr double kFarSideCargoBayX = kCenterFieldX - InchToMeters(20.875);
  constexpr double kMidSideCargoBayX = kFarSideCargoBayX - InchToMeters(21.75);
  constexpr double kNearSideCargoBayX = kMidSideCargoBayX - InchToMeters(21.75);
  constexpr double kSideCargoBayY = InchToMeters(24 + 3 + 0.875);
  constexpr double kSideCargoBayTheta = -M_PI_2;

  constexpr double kFaceCargoBayX =
      kCenterFieldX - InchToMeters(7 * 12 + 11.75 + 9);
  constexpr double kFaceCargoBayY = InchToMeters(10.875);
  constexpr double kFaceCargoBayTheta = 0.0;

  constexpr double kRocketX = kCenterFieldX - FeetToMeters(8);
  constexpr double kRocketY = InchToMeters((26 * 12 + 10.5) / 2.0);

  constexpr double kRocketPortX = kRocketX;
  constexpr double kRocketPortY = kRocketY - 0.70;
  constexpr double kRocketPortTheta = M_PI_2;

  // Half of portal + guess at width * cos(61.5 deg)
  const double kRocketHatchXOffset = InchToMeters(14.634);
  const double kRocketHatchY = kRocketPortY + InchToMeters(9.326);
  const double kRocketNearX = kRocketX - kRocketHatchXOffset;
  const double kRocketFarX = kRocketX + kRocketHatchXOffset;
  constexpr double kRocketNearTheta = DegToRad(28.5);
  constexpr double kRocketFarTheta = M_PI - kRocketNearTheta;

  constexpr double kHpSlotY = InchToMeters((26 * 12 + 10.5) / 2.0 - 25.9);
  constexpr double kHpSlotTheta = M_PI;

  constexpr double kNormalZ = 0.80;
  constexpr double kPortZ = 1.00;

  constexpr double kDiscRadius = InchToMeters(19.0 / 2.0);

  constexpr Target::GoalType kBothGoal = Target::GoalType::kBoth;
  constexpr Target::GoalType kBallGoal = Target::GoalType::kBalls;
  constexpr Target::GoalType kDiscGoal = Target::GoalType::kHatches;
  constexpr Target::GoalType kNoneGoal = Target::GoalType::kNone;
  using TargetType = Target::TargetType;

  const Target far_side_cargo_bay(
      {{kFarSideCargoBayX, kSideCargoBayY, kNormalZ}, kSideCargoBayTheta},
      kDiscRadius, TargetType::kFarSideCargoBay, kBothGoal);
  const Target mid_side_cargo_bay(
      {{kMidSideCargoBayX, kSideCargoBayY, kNormalZ}, kSideCargoBayTheta},
      kDiscRadius, TargetType::kMidSideCargoBay, kBothGoal);
  const Target near_side_cargo_bay(
      {{kNearSideCargoBayX, kSideCargoBayY, kNormalZ}, kSideCargoBayTheta},
      kDiscRadius, TargetType::kNearSideCargoBay, kBothGoal);

  const Target face_cargo_bay(
      {{kFaceCargoBayX, kFaceCargoBayY, kNormalZ}, kFaceCargoBayTheta},
      kDiscRadius, TargetType::kFaceCargoBay, kBothGoal);

  // The rocket port, since it is only for balls, has no meaningful radius
  // to work with (and is over-ridden with zero in target_selector).
  const Target rocket_port(
      {{kRocketPortX, kRocketPortY, kPortZ}, kRocketPortTheta}, 0.0,
      TargetType::kRocketPortal, kBallGoal);

  const Target rocket_near(
      {{kRocketNearX, kRocketHatchY, kNormalZ}, kRocketNearTheta}, kDiscRadius,
      TargetType::kNearRocket, kDiscGoal);
  const Target rocket_far(
      {{kRocketFarX, kRocketHatchY, kNormalZ}, kRocketFarTheta}, kDiscRadius,
      TargetType::kFarRocket, kDiscGoal);

  const Target hp_slot({{0.0, kHpSlotY, kNormalZ}, kHpSlotTheta}, 0.00,
                       TargetType::kHPSlot, kBothGoal);

  const ::std::array<Target, 8> quarter_field_targets{
      {far_side_cargo_bay, mid_side_cargo_bay, near_side_cargo_bay,
       face_cargo_bay, rocket_port, rocket_near, rocket_far, hp_slot}};

  // Mirror across center field mid-line (short field axis):
  ::std::array<Target, 16> half_field_targets;
  ::std::copy(quarter_field_targets.begin(), quarter_field_targets.end(),
              half_field_targets.begin());
  for (int ii = 0; ii < 8; ++ii) {
    const int jj = ii + 8;
    half_field_targets[jj] = quarter_field_targets[ii];
    half_field_targets[jj].mutable_pose()->mutable_pos()->x() =
        2.0 * kCenterFieldX - half_field_targets[jj].pose().rel_pos().x();
    half_field_targets[jj].mutable_pose()->set_theta(aos::math::NormalizeAngle(
        M_PI - half_field_targets[jj].pose().rel_theta()));
    // Targets on the opposite side of the field can't be driven to.
    half_field_targets[jj].set_goal_type(kNoneGoal);
  }

  // Mirror across x-axis (long field axis):
  ::std::copy(half_field_targets.begin(), half_field_targets.end(),
              targets_.begin());
  for (int ii = 0; ii < 16; ++ii) {
    const int jj = ii + 16;
    targets_[jj] = half_field_targets[ii];
    targets_[jj].mutable_pose()->mutable_pos()->y() *= -1;
    targets_[jj].mutable_pose()->set_theta(-targets_[jj].pose().rel_theta());
  }

  // Define rocket obstacles as just being a single line that should block any
  // cameras trying to see through the rocket up and down the field.
  // This line is parallel to the driver's station wall and extends behind
  // the portal.
  Obstacle rocket_obstacle({{kRocketPortX, kRocketY, 0.0}, 0.0},
                           {{kRocketPortX, kRocketPortY + 0.01, 0.0}, 0.0});
  // First, we mirror rocket obstacles across x-axis:
  Obstacle rocket_obstacle2({{kRocketPortX, -kRocketY, 0.0}, 0.0},
                            {{kRocketPortX, -kRocketPortY - 0.01, 0.0}, 0.0});

  // Define an obstacle for the Hab that extends striaght out a few feet from
  // the driver's station wall.
  // TODO(james): Does this actually block our view?
  const double kHabL3X = FeetToMeters(4.0);
  Obstacle hab_obstacle({}, {{kHabL3X, 0.0, 0.0}, 0.0});
  ::std::array<Obstacle, 3> half_obstacles{
      {rocket_obstacle, rocket_obstacle2, hab_obstacle}};
  ::std::copy(half_obstacles.begin(), half_obstacles.end(), obstacles_.begin());

  // Next, we mirror across the mid-line (short axis) to duplicate the
  // rockets and hab to opposite side of the field.
  for (int ii = 0; ii < 3; ++ii) {
    const int jj = ii + 3;
    obstacles_[jj] = half_obstacles[ii];
    obstacles_[jj].mutable_pose1()->mutable_pos()->x() =
        2.0 * kCenterFieldX - obstacles_[jj].mutable_pose1()->rel_pos().x();
    obstacles_[jj].mutable_pose2()->mutable_pos()->x() =
        2.0 * kCenterFieldX - obstacles_[jj].mutable_pose2()->rel_pos().x();
  }

  // Finally, define a rectangular cargo ship.
  const double kCargoCornerX = kFaceCargoBayX + 0.1;
  const double kCargoCornerY = kSideCargoBayY - 0.1;
  ::std::array<Pose, 4> cargo_corners{
      {{{kCargoCornerX, kCargoCornerY, 0.0}, 0.0},
       {{kCargoCornerX, -kCargoCornerY, 0.0}, 0.0},
       {{2.0 * kCenterFieldX - kCargoCornerX, -kCargoCornerY, 0.0}, 0.0},
       {{2.0 * kCenterFieldX - kCargoCornerX, kCargoCornerY, 0.0}, 0.0}}};
  for (int ii = 6; ii < 10; ++ii) {
    obstacles_[ii] = Obstacle(cargo_corners[ii % cargo_corners.size()],
                              cargo_corners[(ii + 1) % cargo_corners.size()]);
  }
}

}  // namespace constants
}  // namespace y2019
