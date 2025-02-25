#include "y2025/localizer/localizer.h"

#include <chrono>

#include "absl/flags/flag.h"

#include "aos/time/time.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/vision/target_map_utils.h"

ABSL_FLAG(bool, use_swerve_extended_kalman_filter, false,
          "If true it uses the output of the ekf from the swerve velocity "
          "estimation.");
ABSL_FLAG(double, max_pose_error, 1e-5,
          "Throw out targets with a higher pose error than this.");
ABSL_FLAG(double, max_pose_error_ratio, 0.4,
          "Throw out targets with a higher pose error ratio than this.");

namespace y2025::localizer {
namespace {

using States =
    frc971::control_loops::swerve::SimplifiedDynamics<double>::States;

constexpr std::array<std::string_view, Localizer::kNumCameras>
    kDetectionChannels{"/orin1/camera0", "/orin1/camera1", "/imu/camera0",
                       "/imu/camera1"};

size_t CameraIndexForName(std::string_view name) {
  auto name_iter =
      std::find(kDetectionChannels.begin(), kDetectionChannels.end(), name);
  if (name_iter == kDetectionChannels.end()) {
    LOG(FATAL) << "No camera channel named " << name;
  }
  return name_iter - kDetectionChannels.begin();
}

std::map<uint64_t, Localizer::Transform> GetTargetLocations(
    const Constants &constants) {
  CHECK(constants.has_common());
  CHECK(constants.common()->has_target_map());
  CHECK(constants.common()->target_map()->has_target_poses());
  std::map<uint64_t, Localizer::Transform> transforms;
  for (const frc971::vision::TargetPoseFieldFbs *target :
       *constants.common()->target_map()->target_poses()) {
    CHECK(target->has_id());
    CHECK(target->has_position());
    CHECK(target->has_orientation());
    CHECK_EQ(0u, transforms.count(target->id()));
    transforms[target->id()] = PoseToTransform(target);
  }
  return transforms;
}

std::array<Localizer::CameraState, Localizer::kNumCameras> MakeCameras(
    const Constants &constants) {
  CHECK(constants.has_cameras());

  std::array<Localizer::CameraState, Localizer::kNumCameras> cameras;

  for (const CameraConfiguration *camera : *constants.cameras()) {
    CHECK(camera->has_calibration());
    const frc971::vision::calibration::CameraCalibration *calibration =
        camera->calibration();

    CHECK(!calibration->has_turret_extrinsics())
        << "The 2025 robot does not have cameras on a turret.";
    CHECK(calibration->has_node_name());

    const std::string channel_name =
        absl::StrFormat("/%s/camera%d", calibration->node_name()->string_view(),
                        calibration->camera_number());

    const size_t index = CameraIndexForName(channel_name);

    // We default-construct the extrinsics matrix to all-zeros; use that to
    // sanity-check whether we have populated the matrix yet or not.
    CHECK(cameras.at(index).extrinsics.norm() == 0)
        << "Got multiple calibrations for "
        << calibration->node_name()->string_view();
    CHECK(calibration->has_fixed_extrinsics());

    cameras.at(index).extrinsics =
        frc971::control_loops::drivetrain::FlatbufferToTransformationMatrix(
            *calibration->fixed_extrinsics());
  }
  for (const Localizer::CameraState &camera : cameras) {
    CHECK(camera.extrinsics.norm() != 0) << "Missing a camera calibration.";
  }
  return cameras;
}

void PopulateCameraStats(const Localizer::CameraState &camera,
                         CameraStatsStatic *stats) {
  camera.rejection_counter.PopulateCountsStaticFbs(stats->add_rejections());

  stats->set_total_accepted(camera.total_accepted_targets);
  stats->set_total_candidates(camera.total_candidate_targets);
}

}  // namespace
Localizer::Localizer(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      constants_fetcher_(event_loop_),
      localizer_state_sender_(
          event_loop_->MakeSender<LocalizerStateStatic>("/localizer")),
      localizer_status_sender_(
          event_loop_->MakeSender<StatusStatic>("/localizer")),
      cameras_(MakeCameras(constants_fetcher_.constants())),
      target_poses_(GetTargetLocations(constants_fetcher_.constants())),
      utils_(event_loop) {
  for (size_t camera_index = 0; camera_index < Localizer::kNumCameras;
       ++camera_index) {
    const std::string_view channel_name = kDetectionChannels.at(camera_index);
    const aos::Channel *const channel =
        event_loop_->GetChannel<frc971::vision::TargetMap>(channel_name);

    CHECK(channel != nullptr);

    event_loop_->MakeWatcher(
        channel_name, [this, channel,
                       camera_index](const frc971::vision::TargetMap &targets) {
          CHECK(targets.has_target_poses());
          CHECK(targets.has_monotonic_timestamp_ns());

          const std::optional<aos::monotonic_clock::duration> clock_offset =
              utils_.ClockOffset(channel->source_node()->string_view());
          if (!clock_offset.has_value()) {
            VLOG(1) << "Rejecting image due to disconnected message bridge at "
                    << event_loop_->monotonic_now();

            cameras_.at(camera_index)
                .rejection_counter.IncrementError(
                    RejectionReason::MESSAGE_BRIDGE_DISCONNECTED);
            return;
          }

          const aos::monotonic_clock::time_point orin_capture_time(
              std::chrono::nanoseconds(targets.monotonic_timestamp_ns()) -
              clock_offset.value());

          if (orin_capture_time > event_loop_->context().monotonic_event_time) {
            VLOG(1) << "Rejecting image due to being from future at "
                    << event_loop_->monotonic_now() << " with timestamp of "
                    << orin_capture_time << " and event time pf "
                    << event_loop_->context().monotonic_event_time;

            cameras_.at(camera_index)
                .rejection_counter.IncrementError(
                    RejectionReason::IMAGE_FROM_THE_FUTURE);
            return;
          }

          for (const frc971::vision::TargetPoseFbs *target :
               *targets.target_poses()) {
            HandleTargetPoseFbs(target, camera_index, orin_capture_time,
                                event_loop_->context().monotonic_event_time);
          }
        });
  }

  event_loop->MakeWatcher(
      "/drivetrain",
      [this](const frc971::control_loops::swerve::AutonomousInit &init) {
        HandleAutonomousInit(init);
      });

  event_loop->MakeWatcher(
      "/drivetrain",
      [this](const frc971::control_loops::swerve::Status &status) {
        HandleSwerveStatus(status, event_loop_->context().monotonic_event_time);
      });

  event_loop->AddPhasedLoop(
      [this](int) {
        SendOutput(localizer_state_sender_.MakeStaticBuilder());
        SendStatus();
      },
      std::chrono::milliseconds(10));

  // TODO(max): This was taken from last year, we should probably fix it.
  event_loop->SetRuntimeRealtimePriority(10);
}

// Converts the camera to target pose we get from our detection to a robot to
// field pose. We can do this because we know the transformation from the robot
// to the camera (H_robot_camera) and the transformation from the field to the
// target (H_field_target).
void Localizer::HandleTargetPoseFbs(
    const frc971::vision::TargetPoseFbs *target, size_t camera_index,
    aos::monotonic_clock::time_point capture_time,
    aos::monotonic_clock::time_point now) {
  CameraState camera = cameras_.at(camera_index);
  const uint64_t target_id = target->id();

  std::set<int> accepted_targets{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  if (!accepted_targets.contains(target_id)) {
    return;
  }

  if (!target_poses_.contains(target_id)) {
    VLOG(1) << "Rejecting target to invalid ID " << target_id;
    camera.rejection_counter.IncrementError(RejectionReason::INVALID_TAG_ID);
    return;
  }

  if (target->pose_error() > absl::GetFlag(FLAGS_max_pose_error)) {
    VLOG(1) << "Rejecting target due to high pose error "
            << target->pose_error() << " which is greater than "
            << absl::GetFlag(FLAGS_max_pose_error);
    camera.rejection_counter.IncrementError(RejectionReason::HIGH_POSE_ERROR);
    return;
  }

  if (target->pose_error_ratio() > absl::GetFlag(FLAGS_max_pose_error_ratio)) {
    VLOG(1) << "Rejecting target due to high pose error ratio "
            << target->pose_error_ratio() << " which is greater than "
            << absl::GetFlag(FLAGS_max_pose_error_ratio);
    camera.rejection_counter.IncrementError(
        RejectionReason::HIGH_POSE_ERROR_RATIO);
    return;
  }

  const Transform &H_field_target = target_poses_.at(target_id);

  const Transform &H_robot_camera = camera.extrinsics;

  const Transform H_camera_target = PoseToTransform(target);

  const Transform H_robot_target = H_robot_camera * H_camera_target;

  const Transform H_field_robot = H_field_target * H_robot_target.inverse();

  const Pose robot_position(H_field_robot);

  XYThetaPose robot_pose;

  robot_pose << robot_position.abs_pos()(0), robot_position.abs_pos()(1),
      (robot_position.abs_theta() - (M_PI / 2.0));

  debug_estimates_.push_back(DebugPoseEstimate{
      .camera = camera_index,
      .april_tag = target_id,
      .robot_pose = robot_pose,
  });

  HandleDetectedRobotPose(robot_pose, Pose(H_robot_target).xy_norm(), target_id,
                          target->distortion_factor(), capture_time, now);
}

void Localizer::HandleSwerveStatus(
    const frc971::control_loops::swerve::Status &status,
    aos::monotonic_clock::time_point now) {
  CHECK(status.monotonic_timestamp_ns());
  std::optional<Eigen::Matrix<double, States::kNumVelocityStates, 1>> x_hat;

  if (absl::GetFlag(FLAGS_use_swerve_extended_kalman_filter)) {
    x_hat = frc971::ToEigenOrDie<States::kNumVelocityStates, 1>(
        *status.velocity_ekf()->x_hat());
  } else {
    x_hat = frc971::ToEigenOrDie<States::kNumVelocityStates, 1>(
        *status.naive_estimator()->x_hat());
  }

  CHECK(x_hat.has_value()) << "Did not recieve an x_hat from either "
                              "naive_estimator or velocity_ekf.";

  if (!last_estimated_velocity_timestamp.has_value()) {
    last_estimated_velocity_timestamp = aos::monotonic_clock::time_point(
        aos::monotonic_clock::duration(status.monotonic_timestamp_ns()));
    return;
  }

  aos::monotonic_clock::duration dt =
      aos::monotonic_clock::time_point(
          aos::monotonic_clock::duration(status.monotonic_timestamp_ns())) -
      last_estimated_velocity_timestamp.value();

  last_estimated_velocity_timestamp = aos::monotonic_clock::time_point(
      aos::monotonic_clock::duration(status.monotonic_timestamp_ns()));

  std::chrono::duration<double, std::ratio<1, 1>> dt_sec = dt;

  estimated_pose_(PoseIdx::kX) += x_hat.value()(States::kVx) * dt_sec.count();
  estimated_pose_(PoseIdx::kY) += x_hat.value()(States::kVy) * dt_sec.count();
  estimated_pose_(PoseIdx::kTheta) +=
      x_hat.value()(States::kOmega) * dt_sec.count();

  HandleEstimatedSwerveState(
      EstimatedSwerveState{
          .x = estimated_pose_(PoseIdx::kX),
          .y = estimated_pose_(PoseIdx::kY),
          .theta = estimated_pose_(PoseIdx::kTheta),
          .vx = x_hat.value()(States::kVx),
          .vy = x_hat.value()(States::kVy),
          .omega = x_hat.value()(States::kOmega),
      },
      now);
}

void Localizer::SendStatus() {
  auto status_builder = localizer_status_sender_.MakeStaticBuilder();

  StatsStatic *stats = status_builder->add_stats();

  PopulateCameraStats(cameras_.at(CameraIndexForName("/orin1/camera0")),
                      stats->add_orin1_camera0());
  PopulateCameraStats(cameras_.at(CameraIndexForName("/orin1/camera1")),
                      stats->add_orin1_camera1());
  PopulateCameraStats(cameras_.at(CameraIndexForName("/imu/camera0")),
                      stats->add_imu_camera0());
  PopulateCameraStats(cameras_.at(CameraIndexForName("/imu/camera1")),
                      stats->add_imu_camera1());
  auto *estimates = status_builder->add_debug_estimates();
  for (const DebugPoseEstimate &estimate : debug_estimates_) {
    DebugPoseEstimateStatic *fb_estimate = estimates->emplace_back();

    fb_estimate->set_camera(estimate.camera);
    fb_estimate->set_april_tag(estimate.april_tag);
    fb_estimate->set_robot_x(estimate.robot_pose(PoseIdx::kX));
    fb_estimate->set_robot_y(estimate.robot_pose(PoseIdx::kY));
    fb_estimate->set_robot_theta(estimate.robot_pose(PoseIdx::kTheta));
  }

  debug_estimates_.clear();

  status_builder.CheckOk(status_builder.Send());
}

}  // namespace y2025::localizer
