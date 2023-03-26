#include "y2023/localizer/localizer.h"

#include "aos/containers/sized_array.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/pose.h"
#include "gflags/gflags.h"
#include "y2023/constants.h"
#include "y2023/localizer/utils.h"

DEFINE_double(max_pose_error, 1e-6,
              "Throw out target poses with a higher pose error than this");
DEFINE_double(
    max_pose_error_ratio, 0.4,
    "Throw out target poses with a higher pose error ratio than this");
DEFINE_double(distortion_noise_scalar, 1.0,
              "Scale the target pose distortion factor by this when computing "
              "the noise.");
DEFINE_double(
    max_implied_yaw_error, 3.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
DEFINE_double(
    max_implied_teleop_yaw_error, 30.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
DEFINE_double(max_distance_to_target, 3.5,
              "Reject target poses that have a 3d distance of more than this "
              "many meters.");
DEFINE_double(max_auto_image_robot_speed, 2.0,
              "Reject target poses when the robot is travelling faster than "
              "this speed in auto.");

namespace y2023::localizer {
namespace {
constexpr std::array<std::string_view, Localizer::kNumCameras> kPisToUse{
    "pi1", "pi2", "pi3", "pi4"};

size_t CameraIndexForName(std::string_view name) {
  for (size_t index = 0; index < kPisToUse.size(); ++index) {
    if (name == kPisToUse.at(index)) {
      return index;
    }
  }
  LOG(FATAL) << "No camera named " << name;
}

std::map<uint64_t, Localizer::Transform> GetTargetLocations(
    const Constants &constants) {
  CHECK(constants.has_target_map());
  CHECK(constants.target_map()->has_target_poses());
  std::map<uint64_t, Localizer::Transform> transforms;
  for (const frc971::vision::TargetPoseFbs *target :
       *constants.target_map()->target_poses()) {
    CHECK(target->has_id());
    CHECK(target->has_position());
    CHECK(target->has_orientation());
    CHECK_EQ(0u, transforms.count(target->id()));
    transforms[target->id()] = PoseToTransform(target);
  }
  return transforms;
}
}  // namespace

std::array<Localizer::CameraState, Localizer::kNumCameras>
Localizer::MakeCameras(const Constants &constants, aos::EventLoop *event_loop) {
  CHECK(constants.has_cameras());
  std::array<Localizer::CameraState, Localizer::kNumCameras> cameras;
  for (const CameraConfiguration *camera : *constants.cameras()) {
    CHECK(camera->has_calibration());
    const frc971::vision::calibration::CameraCalibration *calibration =
        camera->calibration();
    CHECK(!calibration->has_turret_extrinsics())
        << "The 2023 robot does not have a turret.";
    CHECK(calibration->has_node_name());
    const size_t index =
        CameraIndexForName(calibration->node_name()->string_view());
    // We default-construct the extrinsics matrix to all-zeros; use that to
    // sanity-check whether we have populated the matrix yet or not.
    CHECK(cameras.at(index).extrinsics.norm() == 0)
        << "Got multiple calibrations for "
        << calibration->node_name()->string_view();
    CHECK(calibration->has_fixed_extrinsics());
    cameras.at(index).extrinsics =
        frc971::control_loops::drivetrain::FlatbufferToTransformationMatrix(
            *calibration->fixed_extrinsics());
    cameras.at(index).debug_sender = event_loop->MakeSender<Visualization>(
        absl::StrCat("/", calibration->node_name()->string_view(), "/camera"));
  }
  for (const CameraState &camera : cameras) {
    CHECK(camera.extrinsics.norm() != 0) << "Missing a camera calibration.";
  }
  return cameras;
}

Localizer::Localizer(
    aos::EventLoop *event_loop,
    const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config)
    : event_loop_(event_loop),
      dt_config_(dt_config),
      constants_fetcher_(event_loop),
      cameras_(MakeCameras(constants_fetcher_.constants(), event_loop)),
      target_poses_(GetTargetLocations(constants_fetcher_.constants())),
      down_estimator_(dt_config),
      ekf_(dt_config),
      observations_(&ekf_),
      imu_watcher_(event_loop, dt_config,
                   y2023::constants::Values::DrivetrainEncoderToMeters(1),
                   std::bind(&Localizer::HandleImu, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3,
                             std::placeholders::_4, std::placeholders::_5),
                   frc971::controls::ImuWatcher::TimestampSource::kPi),
      utils_(event_loop),
      status_sender_(event_loop->MakeSender<Status>("/localizer")),
      output_sender_(event_loop->MakeSender<frc971::controls::LocalizerOutput>(
          "/localizer")) {
  if (dt_config_.is_simulated) {
    down_estimator_.assume_perfect_gravity();
  }

  for (size_t camera_index = 0; camera_index < kNumCameras; ++camera_index) {
    const std::string_view pi_name = kPisToUse.at(camera_index);
    event_loop->MakeWatcher(
        absl::StrCat("/", pi_name, "/camera"),
        [this, pi_name,
         camera_index](const frc971::vision::TargetMap &targets) {
          CHECK(targets.has_target_poses());
          CHECK(targets.has_monotonic_timestamp_ns());
          const std::optional<aos::monotonic_clock::duration> clock_offset =
              utils_.ClockOffset(pi_name);
          if (!clock_offset.has_value()) {
            VLOG(1) << "Rejecting image due to disconnected message bridge at "
                    << event_loop_->monotonic_now();
            cameras_.at(camera_index)
                .rejection_counter.IncrementError(
                    RejectionReason::MESSAGE_BRIDGE_DISCONNECTED);
            return;
          }
          const aos::monotonic_clock::time_point pi_capture_time(
              std::chrono::nanoseconds(targets.monotonic_timestamp_ns()) -
              clock_offset.value());
          const aos::monotonic_clock::time_point capture_time =
              pi_capture_time - imu_watcher_.pico_offset_error();
          VLOG(2) << "Capture time of "
                  << targets.monotonic_timestamp_ns() * 1e-9
                  << " clock offset of "
                  << aos::time::DurationInSeconds(clock_offset.value())
                  << " pico error "
                  << aos::time::DurationInSeconds(
                         imu_watcher_.pico_offset_error());
          if (pi_capture_time > event_loop_->context().monotonic_event_time) {
            VLOG(1) << "Rejecting image due to being from future at "
                    << event_loop_->monotonic_now() << " with timestamp of "
                    << pi_capture_time << " and event time pf "
                    << event_loop_->context().monotonic_event_time;
            cameras_.at(camera_index)
                .rejection_counter.IncrementError(
                    RejectionReason::IMAGE_FROM_FUTURE);
            return;
          }
          auto builder = cameras_.at(camera_index).debug_sender.MakeBuilder();
          aos::SizedArray<flatbuffers::Offset<TargetEstimateDebug>, 20>
              debug_offsets;
          for (const frc971::vision::TargetPoseFbs *target :
               *targets.target_poses()) {
            VLOG(1) << "Handling target from " << camera_index;
            auto offset = HandleTarget(camera_index, capture_time, *target,
                                       builder.fbb());
            if (debug_offsets.size() < debug_offsets.capacity()) {
              debug_offsets.push_back(offset);
            } else {
              AOS_LOG(ERROR, "Dropped message from debug vector.");
            }
          }
          auto vector_offset = builder.fbb()->CreateVector(
              debug_offsets.data(), debug_offsets.size());
          auto stats_offset =
              StatisticsForCamera(cameras_.at(camera_index), builder.fbb());
          Visualization::Builder visualize_builder(*builder.fbb());
          visualize_builder.add_targets(vector_offset);
          visualize_builder.add_statistics(stats_offset);
          builder.CheckOk(builder.Send(visualize_builder.Finish()));
          SendStatus();
        });
  }

  event_loop_->AddPhasedLoop([this](int) { SendOutput(); },
                             std::chrono::milliseconds(20));

  event_loop_->MakeWatcher(
      "/drivetrain",
      [this](
          const frc971::control_loops::drivetrain::LocalizerControl &control) {
        const double theta = control.keep_current_theta()
                                 ? ekf_.X_hat(StateIdx::kTheta)
                                 : control.theta();
        const double left_encoder = ekf_.X_hat(StateIdx::kLeftEncoder);
        const double right_encoder = ekf_.X_hat(StateIdx::kRightEncoder);
        ekf_.ResetInitialState(
            t_,
            (HybridEkf::State() << control.x(), control.y(), theta,
             left_encoder, 0, right_encoder, 0, 0, 0, 0, 0, 0)
                .finished(),
            ekf_.P());
      });

  ekf_.set_ignore_accel(true);
  // Priority should be lower than the imu reading process, but non-zero.
  event_loop->SetRuntimeRealtimePriority(10);
  event_loop->OnRun([this, event_loop]() {
    ekf_.ResetInitialState(event_loop->monotonic_now(),
                           HybridEkf::State::Zero(), ekf_.P());
  });
}

void Localizer::HandleImu(aos::monotonic_clock::time_point sample_time_pico,
                          aos::monotonic_clock::time_point sample_time_pi,
                          std::optional<Eigen::Vector2d> encoders,
                          Eigen::Vector3d gyro, Eigen::Vector3d accel) {
  last_encoder_readings_ = encoders;
  // Ignore ivnalid readings; the HybridEkf will handle it reasonably.
  if (!encoders.has_value()) {
    return;
  }
  if (t_ == aos::monotonic_clock::min_time) {
    t_ = sample_time_pico;
  }
  if (t_ + 10 * frc971::controls::ImuWatcher::kNominalDt < sample_time_pico) {
    t_ = sample_time_pico;
    ++clock_resets_;
  }
  const aos::monotonic_clock::duration dt = sample_time_pico - t_;
  t_ = sample_time_pico;
  // We don't actually use the down estimator currently, but it's really
  // convenient for debugging.
  down_estimator_.Predict(gyro, accel, dt);
  const double yaw_rate = (dt_config_.imu_transform * gyro)(2);
  ekf_.UpdateEncodersAndGyro(encoders.value()(0), encoders.value()(1), yaw_rate,
                             utils_.VoltageOrZero(sample_time_pi), accel, t_);
  SendStatus();
}

flatbuffers::Offset<TargetEstimateDebug> Localizer::RejectImage(
    int camera_index, RejectionReason reason,
    TargetEstimateDebug::Builder *builder) {
  builder->add_accepted(false);
  builder->add_rejection_reason(reason);
  cameras_.at(camera_index).rejection_counter.IncrementError(reason);
  return builder->Finish();
}

flatbuffers::Offset<TargetEstimateDebug> Localizer::HandleTarget(
    int camera_index, const aos::monotonic_clock::time_point capture_time,
    const frc971::vision::TargetPoseFbs &target,
    flatbuffers::FlatBufferBuilder *debug_fbb) {
  ++total_candidate_targets_;
  ++cameras_.at(camera_index).total_candidate_targets;

  TargetEstimateDebug::Builder builder(*debug_fbb);
  builder.add_camera(camera_index);
  builder.add_image_age_sec(aos::time::DurationInSeconds(
      event_loop_->monotonic_now() - capture_time));
  builder.add_image_monotonic_timestamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          capture_time.time_since_epoch())
          .count());

  const uint64_t target_id = target.id();
  builder.add_april_tag(target_id);
  VLOG(2) << aos::FlatbufferToJson(&target);
  if (target_poses_.count(target_id) == 0) {
    VLOG(1) << "Rejecting target due to invalid ID " << target_id;
    return RejectImage(camera_index, RejectionReason::NO_SUCH_TARGET, &builder);
  }

  const Transform &H_field_target = target_poses_.at(target_id);
  const Transform &H_robot_camera = cameras_.at(camera_index).extrinsics;

  const Transform H_camera_target = PoseToTransform(&target);

  const Transform H_field_camera = H_field_target * H_camera_target.inverse();
  // Back out the robot position that is implied by the current camera
  // reading. Note that the Pose object ignores any roll/pitch components, so
  // if the camera's extrinsics for pitch/roll are off, this should just
  // ignore it.
  const frc971::control_loops::Pose measured_camera_pose(H_field_camera);
  builder.add_camera_x(measured_camera_pose.rel_pos().x());
  builder.add_camera_y(measured_camera_pose.rel_pos().y());
  // Because the camera uses Z as forwards rather than X, just calculate the
  // debugging theta value using the transformation matrix directly.
  builder.add_camera_theta(
      std::atan2(H_field_camera(1, 2), H_field_camera(0, 2)));
  // Calculate the camera-to-robot transformation matrix ignoring the
  // pitch/roll of the camera.
  const Transform H_camera_robot_stripped =
      frc971::control_loops::Pose(H_robot_camera)
          .AsTransformationMatrix()
          .inverse();
  const frc971::control_loops::Pose measured_pose(
      measured_camera_pose.AsTransformationMatrix() * H_camera_robot_stripped);
  // This "Z" is the robot pose directly implied by the camera results.
  const Eigen::Matrix<double, 3, 1> Z(measured_pose.rel_pos().x(),
                                      measured_pose.rel_pos().y(),
                                      measured_pose.rel_theta());
  builder.add_implied_robot_x(Z(Corrector::kX));
  builder.add_implied_robot_y(Z(Corrector::kY));
  builder.add_implied_robot_theta(Z(Corrector::kTheta));

  Eigen::AngleAxisd rvec_camera_target(
      Eigen::Affine3d(H_camera_target).rotation());
  // Use y angle (around vertical axis) to compute skew
  double skew = rvec_camera_target.axis().y() * rvec_camera_target.angle();
  builder.add_skew(skew);

  double distance_to_target =
      Eigen::Affine3d(H_camera_target).translation().norm();

  // TODO(james): Tune this. Also, gain schedule for auto mode?
  Eigen::Matrix<double, 3, 1> noises(1.0, 1.0, 0.5);
  noises /= 4.0;
  // Scale noise by the distortion factor for this detection
  noises *= (1.0 + FLAGS_distortion_noise_scalar * target.distortion_factor() *
                       std::exp(distance_to_target));

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R.diagonal() = noises.cwiseAbs2();
  // In order to do the EKF correction, we determine the expected state based
  // on the state at the time the image was captured; however, we insert the
  // correction update itself at the current time. This is technically not
  // quite correct, but saves substantial CPU usage & code complexity by
  // making
  // it so that we don't have to constantly rewind the entire EKF history.
  const std::optional<State> state_at_capture =
      ekf_.LastStateBeforeTime(capture_time);

  if (!state_at_capture.has_value()) {
    VLOG(1) << "Rejecting image due to being too old.";
    return RejectImage(camera_index, RejectionReason::IMAGE_TOO_OLD, &builder);
  } else if (target.pose_error() > FLAGS_max_pose_error) {
    VLOG(1) << "Rejecting target due to high pose error "
            << target.pose_error();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR,
                       &builder);
  } else if (target.pose_error_ratio() > FLAGS_max_pose_error_ratio) {
    VLOG(1) << "Rejecting target due to high pose error ratio "
            << target.pose_error_ratio();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR_RATIO,
                       &builder);
  }

  double theta_at_capture = state_at_capture.value()(StateIdx::kTheta);
  double camera_implied_theta = Z(Corrector::kTheta);
  constexpr double kDegToRad = M_PI / 180.0;

  const double robot_speed =
      (state_at_capture.value()(StateIdx::kLeftVelocity) +
       state_at_capture.value()(StateIdx::kRightVelocity)) /
      2.0;
  const double yaw_threshold =
      (utils_.MaybeInAutonomous() ? FLAGS_max_implied_yaw_error
                                  : FLAGS_max_implied_teleop_yaw_error) *
      kDegToRad;
  if (utils_.MaybeInAutonomous() &&
      (std::abs(robot_speed) > FLAGS_max_auto_image_robot_speed)) {
    return RejectImage(camera_index, RejectionReason::ROBOT_TOO_FAST, &builder);
  } else if (std::abs(aos::math::NormalizeAngle(
                 camera_implied_theta - theta_at_capture)) > yaw_threshold) {
    return RejectImage(camera_index, RejectionReason::HIGH_IMPLIED_YAW_ERROR,
                       &builder);
  } else if (distance_to_target > FLAGS_max_distance_to_target) {
    return RejectImage(camera_index, RejectionReason::HIGH_DISTANCE_TO_TARGET,
                       &builder);
  }

  const Input U = ekf_.MostRecentInput();
  VLOG(1) << "previous state " << ekf_.X_hat().topRows<3>().transpose();
  const State prior_state = ekf_.X_hat();
  // For the correction step, instead of passing in the measurement directly,
  // we pass in (0, 0, 0) as the measurement and then for the expected
  // measurement (Zhat) we calculate the error between the pose implied by
  // the camera measurement and the current estimate of the
  // pose. This doesn't affect any of the math, it just makes the code a bit
  // more convenient to write given the Correct() interface we already have.
  observations_.CorrectKnownH(Eigen::Vector3d::Zero(), &U,
                              Corrector(state_at_capture.value(), Z), R, t_);
  ++total_accepted_targets_;
  ++cameras_.at(camera_index).total_accepted_targets;
  VLOG(1) << "new state " << ekf_.X_hat().topRows<3>().transpose();
  builder.add_correction_x(ekf_.X_hat()(StateIdx::kX) -
                           prior_state(StateIdx::kX));
  builder.add_correction_y(ekf_.X_hat()(StateIdx::kY) -
                           prior_state(StateIdx::kY));
  builder.add_correction_theta(ekf_.X_hat()(StateIdx::kTheta) -
                               prior_state(StateIdx::kTheta));
  builder.add_accepted(true);
  return builder.Finish();
}

Localizer::Output Localizer::Corrector::H(const State &, const Input &) {
  CHECK(Z_.allFinite());
  Eigen::Vector3d Zhat = H_ * state_at_capture_ - Z_;
  // Rewrap angle difference to put it back in range.
  Zhat(2) = aos::math::NormalizeAngle(Zhat(2));
  VLOG(1) << "Zhat " << Zhat.transpose() << " Z_ " << Z_.transpose()
          << " state " << (H_ * state_at_capture_).transpose();
  return Zhat;
}

void Localizer::SendOutput() {
  auto builder = output_sender_.MakeBuilder();
  frc971::controls::LocalizerOutput::Builder output_builder =
      builder.MakeBuilder<frc971::controls::LocalizerOutput>();
  output_builder.add_monotonic_timestamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->context().monotonic_event_time.time_since_epoch())
          .count());
  output_builder.add_x(ekf_.X_hat(StateIdx::kX));
  output_builder.add_y(ekf_.X_hat(StateIdx::kY));
  output_builder.add_theta(ekf_.X_hat(StateIdx::kTheta));
  output_builder.add_zeroed(imu_watcher_.zeroer().Zeroed());
  output_builder.add_image_accepted_count(total_accepted_targets_);
  const Eigen::Quaterniond &orientation =
      Eigen::AngleAxis<double>(ekf_.X_hat(StateIdx::kTheta),
                               Eigen::Vector3d::UnitZ()) *
      down_estimator_.X_hat();
  frc971::controls::Quaternion quaternion;
  quaternion.mutate_x(orientation.x());
  quaternion.mutate_y(orientation.y());
  quaternion.mutate_z(orientation.z());
  quaternion.mutate_w(orientation.w());
  output_builder.add_orientation(&quaternion);
  builder.CheckOk(builder.Send(output_builder.Finish()));
}

flatbuffers::Offset<frc971::control_loops::drivetrain::LocalizerState>
Localizer::PopulateState(const State &X_hat,
                         flatbuffers::FlatBufferBuilder *fbb) {
  frc971::control_loops::drivetrain::LocalizerState::Builder builder(*fbb);
  builder.add_x(X_hat(StateIdx::kX));
  builder.add_y(X_hat(StateIdx::kY));
  builder.add_theta(X_hat(StateIdx::kTheta));
  builder.add_left_velocity(X_hat(StateIdx::kLeftVelocity));
  builder.add_right_velocity(X_hat(StateIdx::kRightVelocity));
  builder.add_left_encoder(X_hat(StateIdx::kLeftEncoder));
  builder.add_right_encoder(X_hat(StateIdx::kRightEncoder));
  builder.add_left_voltage_error(X_hat(StateIdx::kLeftVoltageError));
  builder.add_right_voltage_error(X_hat(StateIdx::kRightVoltageError));
  builder.add_angular_error(X_hat(StateIdx::kAngularError));
  builder.add_longitudinal_velocity_offset(
      X_hat(StateIdx::kLongitudinalVelocityOffset));
  builder.add_lateral_velocity(X_hat(StateIdx::kLateralVelocity));
  return builder.Finish();
}

flatbuffers::Offset<ImuStatus> Localizer::PopulateImu(
    flatbuffers::FlatBufferBuilder *fbb) const {
  const auto zeroer_offset = imu_watcher_.zeroer().PopulateStatus(fbb);
  const auto failures_offset = imu_watcher_.PopulateImuFailures(fbb);
  ImuStatus::Builder builder(*fbb);
  builder.add_zeroed(imu_watcher_.zeroer().Zeroed());
  builder.add_faulted_zero(imu_watcher_.zeroer().Faulted());
  builder.add_zeroing(zeroer_offset);
  if (imu_watcher_.pico_offset().has_value()) {
    builder.add_pico_offset_ns(imu_watcher_.pico_offset().value().count());
    builder.add_pico_offset_error_ns(imu_watcher_.pico_offset_error().count());
  }
  if (last_encoder_readings_.has_value()) {
    builder.add_left_encoder(last_encoder_readings_.value()(0));
    builder.add_right_encoder(last_encoder_readings_.value()(1));
  }
  builder.add_imu_failures(failures_offset);
  return builder.Finish();
}

flatbuffers::Offset<CumulativeStatistics> Localizer::StatisticsForCamera(
    const CameraState &camera, flatbuffers::FlatBufferBuilder *fbb) {
  const auto counts_offset = camera.rejection_counter.PopulateCounts(fbb);
  CumulativeStatistics::Builder stats_builder(*fbb);
  stats_builder.add_total_accepted(camera.total_accepted_targets);
  stats_builder.add_total_candidates(camera.total_candidate_targets);
  stats_builder.add_rejection_reasons(counts_offset);
  return stats_builder.Finish();
}

void Localizer::SendStatus() {
  auto builder = status_sender_.MakeBuilder();
  std::array<flatbuffers::Offset<CumulativeStatistics>, kNumCameras>
      stats_offsets;
  for (size_t ii = 0; ii < kNumCameras; ++ii) {
    stats_offsets.at(ii) = StatisticsForCamera(cameras_.at(ii), builder.fbb());
  }
  auto stats_offset =
      builder.fbb()->CreateVector(stats_offsets.data(), stats_offsets.size());
  auto down_estimator_offset =
      down_estimator_.PopulateStatus(builder.fbb(), t_);
  auto imu_offset = PopulateImu(builder.fbb());
  auto state_offset = PopulateState(ekf_.X_hat(), builder.fbb());
  Status::Builder status_builder = builder.MakeBuilder<Status>();
  status_builder.add_state(state_offset);
  status_builder.add_down_estimator(down_estimator_offset);
  status_builder.add_imu(imu_offset);
  status_builder.add_statistics(stats_offset);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

}  // namespace y2023::localizer
