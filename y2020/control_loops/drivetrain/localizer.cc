#include "y2020/control_loops/drivetrain/localizer.h"

#include "y2020/constants.h"

namespace y2020 {
namespace control_loops {
namespace drivetrain {

namespace {
// Converts a flatbuffer TransformationMatrix to an Eigen matrix. Technically,
// this should be able to do a single memcpy, but the extra verbosity here seems
// appropriate.
Eigen::Matrix<float, 4, 4> FlatbufferToTransformationMatrix(
    const frc971::vision::sift::TransformationMatrix &flatbuffer) {
  CHECK_EQ(16u, CHECK_NOTNULL(flatbuffer.data())->size());
  Eigen::Matrix<float, 4, 4> result;
  result.setIdentity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result(row, col) = (*flatbuffer.data())[row * 4 + col];
    }
  }
  return result;
}

// Indices of the pis to use.
const std::array<std::string, 5> kPisToUse{"pi1", "pi2", "pi3", "pi4", "pi5"};

// Calculates the pose implied by the camera target, just based on
// distance/heading components.
Eigen::Vector3f CalculateImpliedPose(const Localizer::State &X,
                                     const Eigen::Matrix4f &H_field_target,
                                     const Localizer::Pose &pose_robot_target) {
  // This code overrides the pose sent directly from the camera code and
  // effectively distills it down to just a distance + heading estimate, on
  // the presumption that these signals will tend to be much lower noise and
  // better-conditioned than other portions of the robot pose.
  // As such, this code assumes that the current estimate of the robot
  // heading is correct and then, given the heading from the camera to the
  // target and the distance from the camera to the target, calculates the
  // position that the robot would have to be at to make the current camera
  // heading + distance correct. This X/Y implied robot position is then
  // used as the measurement in the EKF, rather than the X/Y that is
  // directly returned from the vision processing. This means that
  // the cameras will not correct any drift in the robot heading estimate
  // but will compensate for X/Y position in a way that prioritizes keeping
  // an accurate distance + heading to the goal.

  // Calculate the heading to the robot in the target's coordinate frame.
  const float implied_heading_from_target = aos::math::NormalizeAngle(
      pose_robot_target.heading() + M_PI + X(Localizer::StateIdx::kTheta));
  const float implied_distance = pose_robot_target.xy_norm();
  const Eigen::Vector4f robot_pose_in_target_frame(
      implied_distance * std::cos(implied_heading_from_target),
      implied_distance * std::sin(implied_heading_from_target), 0, 1);
  const Eigen::Vector4f implied_pose =
      H_field_target * robot_pose_in_target_frame;
  return implied_pose.topRows<3>();
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
  // TODO(james): The down estimator has trouble handling situations where the
  // robot is constantly wiggling but not actually moving much, and can cause
  // drift when using accelerometer readings.
  ekf_.set_ignore_accel(true);
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

  event_loop->OnRun([this, event_loop]() {
    ekf_.ResetInitialState(event_loop->monotonic_now(),
                           HybridEkf::State::Zero(), ekf_.P());
  });

  for (const auto &pi : kPisToUse) {
    image_fetchers_.emplace_back(
        event_loop_->MakeFetcher<frc971::vision::sift::ImageMatchResult>(
            "/" + pi + "/camera"));
  }

  target_selector_.set_has_target(false);
}

void Localizer::Reset(
    aos::monotonic_clock::time_point t,
    const frc971::control_loops::drivetrain::HybridEkf<double>::State &state) {
  // Go through and clear out all of the fetchers so that we don't get behind.
  for (auto &fetcher : image_fetchers_) {
    fetcher.Fetch();
  }
  ekf_.ResetInitialState(t, state.cast<float>(), ekf_.P());
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

void Localizer::Update(const Eigen::Matrix<double, 2, 1> &U,
                       aos::monotonic_clock::time_point now,
                       double left_encoder, double right_encoder,
                       double gyro_rate, const Eigen::Vector3d &accel) {
  ekf_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate,
                             U.cast<float>(), accel.cast<float>(), now);
  for (size_t ii = 0; ii < kPisToUse.size(); ++ii) {
    auto &image_fetcher = image_fetchers_[ii];
    while (image_fetcher.FetchNext()) {
      HandleImageMatch(kPisToUse[ii], *image_fetcher, now);
    }
  }
}

void Localizer::HandleImageMatch(
    std::string_view pi, const frc971::vision::sift::ImageMatchResult &result,
    aos::monotonic_clock::time_point now) {
  std::chrono::nanoseconds monotonic_offset(0);
  clock_offset_fetcher_.Fetch();
  if (clock_offset_fetcher_.get() != nullptr) {
    for (const auto connection : *clock_offset_fetcher_->connections()) {
      if (connection->has_node() && connection->node()->has_name() &&
          connection->node()->name()->string_view() == pi) {
        monotonic_offset =
            std::chrono::nanoseconds(connection->monotonic_offset());
        break;
      }
    }
  }
  aos::monotonic_clock::time_point capture_time(
      std::chrono::nanoseconds(result.image_monotonic_timestamp_ns()) -
      monotonic_offset);
  VLOG(1) << "Got monotonic offset of "
          << aos::time::DurationInSeconds(monotonic_offset)
          << " when at time of " << now << " and capture time estimate of "
          << capture_time;
  if (capture_time > now) {
    LOG(WARNING) << "Got camera frame from the future.";
    return;
  }
  if (!result.has_camera_calibration()) {
    LOG(WARNING) << "Got camera frame without calibration data.";
    return;
  }
  // Per the ImageMatchResult specification, we can actually determine whether
  // the camera is the turret camera just from the presence of the
  // turret_extrinsics member.
  const bool is_turret = result.camera_calibration()->has_turret_extrinsics();
  const TurretData turret_data = GetTurretDataForTime(capture_time);
  // Ignore readings when the turret is spinning too fast, on the assumption
  // that the odds of screwing up the time compensation are higher.
  // Note that the current number here is chosen pretty arbitrarily--1 rad / sec
  // seems reasonable, but may be unnecessarily low or high.
  constexpr float kMaxTurretVelocity = 1.0;
  if (is_turret && std::abs(turret_data.velocity) > kMaxTurretVelocity) {
    return;
  }
  CHECK(result.camera_calibration()->has_fixed_extrinsics());
  const Eigen::Matrix<float, 4, 4> fixed_extrinsics =
      FlatbufferToTransformationMatrix(
          *result.camera_calibration()->fixed_extrinsics());

  // Calculate the pose of the camera relative to the robot origin.
  Eigen::Matrix<float, 4, 4> H_robot_camera = fixed_extrinsics;
  if (is_turret) {
    H_robot_camera = H_robot_camera *
                     frc971::control_loops::TransformationMatrixForYaw<float>(
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
    const Eigen::Matrix<float, 4, 4> H_camera_target =
        FlatbufferToTransformationMatrix(*vision_result->camera_to_target());

    const Eigen::Matrix<float, 4, 4> H_field_target =
        FlatbufferToTransformationMatrix(*vision_result->field_to_target());
    // Back out the robot position that is implied by the current camera
    // reading.
    const Pose measured_pose(H_field_target *
                             (H_robot_camera * H_camera_target).inverse());
    // This "Z" is the robot pose directly implied by the camera results.
    // Currently, we do not actually use this result directly. However, it is
    // kept around in case we want to quickly re-enable it.
    const Eigen::Matrix<float, 3, 1> Z(measured_pose.rel_pos().x(),
                                       measured_pose.rel_pos().y(),
                                       measured_pose.rel_theta());
    // Pose of the target in the robot frame.
    Pose pose_robot_target(H_robot_camera * H_camera_target);
    // TODO(james): Figure out how to properly handle calculating the
    // noise. Currently, the values are deliberately tuned so that image updates
    // will not be trusted overly much. In theory, we should probably also be
    // populating some cross-correlation terms.
    // Note that these are the noise standard deviations (they are squared below
    // to get variances).
    Eigen::Matrix<float, 3, 1> noises(2.0, 2.0, 0.2);
    // Augment the noise by the approximate rotational speed of the
    // camera. This should help account for the fact that, while we are
    // spinning, slight timing errors in the camera/turret data will tend to
    // have mutch more drastic effects on the results.
    noises *= 1.0 + std::abs((right_velocity() - left_velocity()) /
                                 (2.0 * dt_config_.robot_radius) +
                             (is_turret ? turret_data.velocity : 0.0));
    Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
    R.diagonal() = noises.cwiseAbs2();
    Eigen::Matrix<float, HybridEkf::kNOutputs, HybridEkf::kNStates> H;
    H.setZero();
    H(0, StateIdx::kX) = 1;
    H(1, StateIdx::kY) = 1;
    // This is currently set to zero because we ignore the heading implied by
    // the camera.
    H(2, StateIdx::kTheta) = 0;
    VLOG(1) << "Pose implied by target: " << Z.transpose()
            << " and current pose " << x() << ", " << y() << ", " << theta()
            << " Heading/dist/skew implied by target: "
            << pose_robot_target.ToHeadingDistanceSkew().transpose();
    // If the heading is off by too much, assume that we got a false-positive
    // and don't use the correction.
    if (std::abs(aos::math::DiffAngle<float>(theta(), Z(2))) > M_PI_2) {
      AOS_LOG(WARNING, "Dropped image match due to heading mismatch.\n");
      continue;
    }
    // In order to do the EKF correction, we determine the expected state based
    // on the state at the time the image was captured; however, we insert the
    // correction update itself at the current time. This is technically not
    // quite correct, but saves substantial CPU usage by making it so that we
    // don't have to constantly rewind the entire EKF history.
    const std::optional<State> state_at_capture =
        ekf_.LastStateBeforeTime(capture_time);
    if (!state_at_capture.has_value()) {
      AOS_LOG(WARNING, "Dropped image match due to age of image.\n");
      continue;
    }
    const Input U = ekf_.MostRecentInput();
    // For the correction step, instead of passing in the measurement directly,
    // we pass in (0, 0, 0) as the measurement and then for the expected
    // measurement (Zhat) we calculate the error between the implied and actual
    // poses. This doesn't affect any of the math, it just makes the code a bit
    // more convenient to write given the Correct() interface we already have.
    ekf_.Correct(
        Eigen::Vector3f::Zero(), &U, {},
        [H, H_field_target, pose_robot_target, state_at_capture](
            const State &, const Input &) -> Eigen::Vector3f {
          const Eigen::Vector3f Z = CalculateImpliedPose(
              state_at_capture.value(), H_field_target, pose_robot_target);
          // Just in case we ever do encounter any, drop measurements if they
          // have non-finite numbers.
          if (!Z.allFinite()) {
            AOS_LOG(WARNING, "Got measurement with infinites or NaNs.\n");
            return Eigen::Vector3f::Zero();
          }
          Eigen::Vector3f Zhat = H * state_at_capture.value() - Z;
          // Rewrap angle difference to put it back in range. Note that this
          // component of the error is currently ignored (see definition of H
          // above).
          Zhat(2) = aos::math::NormalizeAngle(Zhat(2));
          // If the measurement implies that we are too far from the current
          // estimate, then ignore it.
          // Note that I am not entirely sure how much effect this actually has,
          // because I primarily introduced it to make sure that any grossly
          // invalid measurements get thrown out.
          if (Zhat.squaredNorm() > std::pow(10.0, 2)) {
            return Eigen::Vector3f::Zero();
          }
          return Zhat;
        },
        [H](const State &) { return H; }, R, now);
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020
