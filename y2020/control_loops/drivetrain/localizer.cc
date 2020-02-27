#include "y2020/control_loops/drivetrain/localizer.h"

#include "y2020/constants.h"

namespace y2020 {
namespace control_loops {
namespace drivetrain {

namespace {
// Converts a flatbuffer TransformationMatrix to an Eigen matrix. Technically,
// this should be able to do a single memcpy, but the extra verbosity here seems
// appropriate.
Eigen::Matrix<double, 4, 4> FlatbufferToTransformationMatrix(
    const frc971::vision::sift::TransformationMatrix &flatbuffer) {
  CHECK_EQ(16u, CHECK_NOTNULL(flatbuffer.data())->size());
  Eigen::Matrix<double, 4, 4> result;
  result.setIdentity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result(row, col) = (*flatbuffer.data())[row * 4 + col];
    }
  }
  return result;
}

}  // namespace

Localizer::Localizer(
    aos::EventLoop *event_loop,
    const frc971::control_loops::drivetrain::DrivetrainConfig<double>
        &dt_config)
    : event_loop_(event_loop),
      dt_config_(dt_config),
      ekf_(dt_config),
      clock_offset_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")) {
  // TODO(james): This doesn't really need to be a watcher; we could just use a
  // fetcher for the superstructure status.
  // This probably should be a Fetcher instead of a Watcher, but this
  // seems simpler for the time being (although technically it should be
  // possible to do everything we need to using just a Fetcher without
  // even maintaining a separate buffer, but that seems overly cute).
  event_loop_->MakeWatcher("/superstructure",
                           [this](const superstructure::Status &status) {
                             HandleSuperstructureStatus(status);
                           });

  image_fetchers_.emplace_back(
      event_loop_->MakeFetcher<frc971::vision::sift::ImageMatchResult>(
          "/pi1/camera"));

  target_selector_.set_has_target(false);
}

void Localizer::HandleSuperstructureStatus(
    const y2020::control_loops::superstructure::Status &status) {
  CHECK(status.has_turret());
  turret_data_.Push({event_loop_->monotonic_now(), status.turret()->position(),
                     status.turret()->velocity()});
}

Localizer::TurretData Localizer::GetTurretDataForTime(
    aos::monotonic_clock::time_point time) {
  if (turret_data_.empty()) {
    return {};
  }

  aos::monotonic_clock::duration lowest_time_error =
      aos::monotonic_clock::duration::max();
  TurretData best_data_match;
  for (const auto &sample : turret_data_) {
    const aos::monotonic_clock::duration time_error =
        std::chrono::abs(sample.receive_time - time);
    if (time_error < lowest_time_error) {
      lowest_time_error = time_error;
      best_data_match = sample;
    }
  }
  return best_data_match;
}

void Localizer::Update(const ::Eigen::Matrix<double, 2, 1> &U,
                       aos::monotonic_clock::time_point now,
                       double left_encoder, double right_encoder,
                       double gyro_rate, const Eigen::Vector3d &accel) {
  for (auto &image_fetcher : image_fetchers_) {
    while (image_fetcher.FetchNext()) {
      HandleImageMatch(*image_fetcher);
    }
  }
  ekf_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, accel,
                             now);
}

void Localizer::HandleImageMatch(
    const frc971::vision::sift::ImageMatchResult &result) {
  std::chrono::nanoseconds monotonic_offset(0);
  clock_offset_fetcher_.Fetch();
  if (clock_offset_fetcher_.get() != nullptr) {
    for (const auto connection : *clock_offset_fetcher_->connections()) {
      if (connection->has_node() && connection->node()->has_name() &&
          connection->node()->name()->string_view() == "pi1") {
        monotonic_offset =
            std::chrono::nanoseconds(connection->monotonic_offset());
        break;
      }
    }
  }
  aos::monotonic_clock::time_point capture_time(
      std::chrono::nanoseconds(result.image_monotonic_timestamp_ns()) -
      monotonic_offset);
  CHECK(result.has_camera_calibration());
  // Per the ImageMatchResult specification, we can actually determine whether
  // the camera is the turret camera just from the presence of the
  // turret_extrinsics member.
  const bool is_turret = result.camera_calibration()->has_turret_extrinsics();
  const TurretData turret_data = GetTurretDataForTime(capture_time);
  // Ignore readings when the turret is spinning too fast, on the assumption
  // that the odds of screwing up the time compensation are higher.
  // Note that the current number here is chosen pretty arbitrarily--1 rad / sec
  // seems reasonable, but may be unnecessarily low or high.
  constexpr double kMaxTurretVelocity = 1.0;
  if (is_turret && std::abs(turret_data.velocity) > kMaxTurretVelocity) {
    return;
  }
  CHECK(result.camera_calibration()->has_fixed_extrinsics());
  const Eigen::Matrix<double, 4, 4> fixed_extrinsics =
      FlatbufferToTransformationMatrix(
          *result.camera_calibration()->fixed_extrinsics());
  // Calculate the pose of the camera relative to the robot origin.
  Eigen::Matrix<double, 4, 4> H_robot_camera = fixed_extrinsics;
  if (is_turret) {
    H_robot_camera = H_robot_camera *
                     frc971::control_loops::TransformationMatrixForYaw(
                         turret_data.position) *
                     FlatbufferToTransformationMatrix(
                         *result.camera_calibration()->turret_extrinsics());
  }

  if (!result.has_camera_poses()) {
    return;
  }

  for (const frc971::vision::sift::CameraPose *vision_result :
       *result.camera_poses()) {
    if (!vision_result->has_camera_to_target() ||
        !vision_result->has_field_to_target()) {
      continue;
    }
    const Eigen::Matrix<double, 4, 4> H_camera_target =
        FlatbufferToTransformationMatrix(*vision_result->camera_to_target());
    const Eigen::Matrix<double, 4, 4> H_field_target =
        FlatbufferToTransformationMatrix(*vision_result->field_to_target());
    // Back out the robot position that is implied by the current camera
    // reading.
    const Pose measured_pose(H_field_target *
                             (H_robot_camera * H_camera_target).inverse());
    const Eigen::Matrix<double, 3, 1> Z(measured_pose.rel_pos().x(),
                                        measured_pose.rel_pos().y(),
                                        measured_pose.rel_theta());
    // TODO(james): Figure out how to properly handle calculating the
    // noise. Currently, the values are deliberately tuned so that image updates
    // will not be trusted overly much. In theory, we should probably also be
    // populating some cross-correlation terms.
    // Note that these are the noise standard deviations (they are squared below
    // to get variances).
    Eigen::Matrix<double, 3, 1> noises(1.0, 1.0, 0.1);
    // Augment the noise by the approximate rotational speed of the
    // camera. This should help account for the fact that, while we are
    // spinning, slight timing errors in the camera/turret data will tend to
    // have mutch more drastic effects on the results.
    noises *= 1.0 + std::abs((right_velocity() - left_velocity()) /
                                 (2.0 * dt_config_.robot_radius) +
                             (is_turret ? turret_data.velocity : 0.0));
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R.diagonal() = noises.cwiseAbs2();
    Eigen::Matrix<double, HybridEkf::kNOutputs, HybridEkf::kNStates> H;
    H.setZero();
    H(0, StateIdx::kX) = 1;
    H(1, StateIdx::kY) = 1;
    H(2, StateIdx::kTheta) = 1;
    ekf_.Correct(Z, nullptr, {}, [H, Z](const State &X, const Input &) {
                                   Eigen::Vector3d Zhat = H * X;
                                   // In order to deal with wrapping of the
                                   // angle, calculate an expected angle that is
                                   // in the range (Z(2) - pi, Z(2) + pi].
                                   const double angle_error =
                                       aos::math::NormalizeAngle(
                                           X(StateIdx::kTheta) - Z(2));
                                   Zhat(2) = Z(2) + angle_error;
                                   return Zhat;
                                 },
                 [H](const State &) { return H; }, R, capture_time);
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020
