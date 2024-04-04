#include "y2024/localizer/localizer.h"

#include "gflags/gflags.h"

#include "aos/containers/sized_array.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/math/flatbuffers_matrix.h"
#include "frc971/vision/target_map_utils.h"
#include "y2024/constants.h"

DEFINE_double(max_pose_error, 1e-5,
              "Throw out target poses with a higher pose error than this");
DEFINE_double(max_distortion, 1000.0, "");
DEFINE_double(
    max_pose_error_ratio, 0.4,
    "Throw out target poses with a higher pose error ratio than this");
DEFINE_double(distortion_noise_scalar, 4.0,
              "Scale the target pose distortion factor by this when computing "
              "the noise.");
DEFINE_double(
    max_implied_yaw_error, 5.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
DEFINE_double(
    max_implied_teleop_yaw_error, 30.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
DEFINE_double(max_distance_to_target, 5.0,
              "Reject target poses that have a 3d distance of more than this "
              "many meters.");
DEFINE_double(max_auto_image_robot_speed, 2.0,
              "Reject target poses when the robot is travelling faster than "
              "this speed in auto.");
DEFINE_bool(
    do_xytheta_corrections, true,
    "If set, uses the x/y/theta corrector rather than a heading/distance/skew "
    "one. This is better conditioned currently, but is theoretically worse due "
    "to not capturing noise effectively.");
DEFINE_bool(
    always_use_extra_tags, true,
    "If set, we will use the \"deweighted\" tags even in auto mode (this "
    "affects april tags whose field positions we do not trust as much).");

namespace y2024::localizer {
namespace {
constexpr std::array<std::string_view, Localizer::kNumCameras>
    kDetectionChannels{"/orin1/camera0", "/orin1/camera1", "/imu/camera0",
                       "/imu/camera1"};

size_t CameraIndexForName(std::string_view name) {
  for (size_t index = 0; index < kDetectionChannels.size(); ++index) {
    if (name == kDetectionChannels.at(index)) {
      return index;
    }
  }
  LOG(FATAL) << "No camera channel named " << name;
}

std::map<uint64_t, Localizer::Transform> GetTargetLocations(
    const Constants &constants) {
  CHECK(constants.has_common());
  CHECK(constants.common()->has_target_map());
  CHECK(constants.common()->target_map()->has_target_poses());
  std::map<uint64_t, Localizer::Transform> transforms;
  for (const frc971::vision::TargetPoseFbs *target :
       *constants.common()->target_map()->target_poses()) {
    CHECK(target->has_id());
    CHECK(target->has_position());
    CHECK(target->has_orientation());
    CHECK_EQ(0u, transforms.count(target->id()));
    transforms[target->id()] = PoseToTransform(target);
  }
  return transforms;
}

// Returns the "nominal" covariance of localizer---i.e., the values to which it
// tends to converge during normal operation. By initializing the localizer's
// covariance this way, we reduce the likelihood that the first few corrections
// we receive will result in insane jumps in robot state.
Eigen::Matrix<double, Localizer::HybridEkf::kNStates,
              Localizer::HybridEkf::kNStates>
NominalCovariance() {
  Eigen::Matrix<double, Localizer::HybridEkf::kNStates,
                Localizer::HybridEkf::kNStates>
      P_transpose;
  // Grabbed from when the robot was in a steady-state.
  P_transpose << 0.00344391020344026, 2.78255540964953e-05,
      -3.44257436790434e-09, 1.57165298196431e-09, 0.0207259965606711,
      1.57165298180587e-09, 0.0207259965606711, 0.054775354511474,
      0.0547753545094318, 3.6435938125014e-13, 0.0136249573295751,
      -1.00705421392865e-05, 2.78255540964953e-05, 0.00107448929200992,
      -7.42495169208041e-08, 1.85634700506266e-11, 0.000244343925617656,
      1.85634874205036e-11, 0.000244343925617656, 0.000645553479721632,
      0.000645553790286344, -3.98991820983687e-11, 0.000160471639203211,
      0.00085437373557969, -3.44257436791122e-09, -7.42495169208033e-08,
      8.84891122456971e-05, 5.60929454430362e-16, -3.19015358072956e-08,
      1.00798618104673e-15, -3.19015357689791e-08, 4.05905848804053e-07,
      -5.37043312466153e-07, 2.59177623699213e-08, -3.54286115799832e-08,
      -2.46295184320124e-07, 1.57165298196416e-09, 1.85634700506217e-11,
      5.60929459005148e-16, 4.99891338811926e-09, 3.59436612693873e-08,
      3.54690022095621e-18, 3.59436612693713e-08, 1.47025442116767e-07,
      1.47035806190949e-07, 4.66877989234937e-08, -1.08209016210542e-08,
      -3.39984473837553e-14, 0.0207259965606711, 0.000244343925617655,
      -3.19015358072649e-08, 3.59436612693935e-08, 0.301240404540565,
      3.59436612690168e-08, 0.301240404540535, 1.05741200222346,
      1.0574120022184, 8.29472747900822e-13, 0.138401597893958,
      -1.43941751907531e-06, 1.57165298180564e-09, 1.85634874205096e-11,
      1.00798617244172e-15, 3.54690020212816e-18, 3.59436612690224e-08,
      4.99891338811924e-09, 3.59436612690048e-08, 1.4703580619019e-07,
      1.47025442115683e-07, -4.66877989234395e-08, -1.0820901621393e-08,
      -3.40023588372784e-14, 0.0207259965606711, 0.000244343925617655,
      -3.19015357689244e-08, 3.59436612693859e-08, 0.301240404540535,
      3.5943661269025e-08, 0.301240404540565, 1.05741200222289,
      1.05741200221897, 8.29752131358864e-13, 0.138401597893958,
      -1.43941751907531e-06, 0.0547753545114739, 0.000645553479721628,
      4.05905848802199e-07, 1.4702544211661e-07, 1.05741200222346,
      1.47035806190016e-07, 1.05741200222289, 5.51003071369415,
      4.85571868991385, 3.11581831710161e-06, 0.388669918077443,
      -2.97795819369728e-06, 0.054775354509432, 0.000645553790286345,
      -5.37043312465425e-07, 1.47035806190839e-07, 1.0574120022184,
      1.47025442115792e-07, 1.05741200221897, 4.85571868991385,
      5.51003071367444, -3.11581462746269e-06, 0.388669918072973,
      -2.97799067538699e-06, 3.64359250152554e-13, -3.98991820983e-11,
      2.5917762369921e-08, 4.66877989234987e-08, 8.2947423101614e-13,
      -4.66877989234886e-08, 8.29754326977491e-13, 3.11581831710969e-06,
      -3.11581462747612e-06, 0.212136173309098, 8.06835372350592e-13,
      8.80190080862899e-12, 0.0136249573295751, 0.000160471639203211,
      -3.54286115799303e-08, -1.08209016210412e-08, 0.138401597893958,
      -1.08209016213997e-08, 0.138401597893958, 0.388669918077444,
      0.388669918072972, 8.06834601598773e-13, 0.187427410345505,
      -1.28632768080328e-06, -1.00705421392865e-05, 0.000854373735579689,
      -2.46295184320122e-07, -3.39984473838037e-14, -1.4394175190755e-06,
      -3.40023588373113e-14, -1.4394175190755e-06, -2.97795819369787e-06,
      -2.9779906753875e-06, 8.80190080900245e-12, -1.28632768080338e-06,
      0.00381653175156393;
  return P_transpose.transpose();
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
        << "The 2024 robot does not have cameras on a turret.";
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
    cameras.at(index).debug_sender =
        event_loop->MakeSender<VisualizationStatic>(channel_name);
  }
  for (const CameraState &camera : cameras) {
    CHECK(camera.extrinsics.norm() != 0) << "Missing a camera calibration.";
  }
  return cameras;
}

Localizer::Localizer(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      constants_fetcher_(event_loop),
      dt_config_(
          frc971::control_loops::drivetrain::DrivetrainConfig<double>::
              FromFlatbuffer(*CHECK_NOTNULL(
                  constants_fetcher_.constants().common()->drivetrain()))),
      cameras_(MakeCameras(constants_fetcher_.constants(), event_loop)),
      target_poses_(GetTargetLocations(constants_fetcher_.constants())),
      down_estimator_(dt_config_),
      // Force the dt to 1 ms (the nominal IMU frequency) since we have observed
      // issues with timing on the orins.
      // TODO(james): Ostensibly, we should be able to use the timestamps from
      // the IMU board itself for exactly this; however, I am currently worried
      // about the impacts of clock drift in using that.
      ekf_(dt_config_, std::chrono::milliseconds(1)),
      observations_(&ekf_),
      xyz_observations_(&ekf_),
      imu_watcher_(event_loop, dt_config_,
                   y2024::constants::Values::DrivetrainEncoderToMeters(1),
                   std::bind(&Localizer::HandleImu, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3,
                             std::placeholders::_4, std::placeholders::_5),
                   frc971::controls::ImuWatcher::TimestampSource::kPi),
      utils_(event_loop),
      status_sender_(event_loop->MakeSender<Status>("/localizer")),
      output_sender_(event_loop->MakeSender<frc971::controls::LocalizerOutput>(
          "/localizer")),
      server_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      client_statistics_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ClientStatistics>(
              "/aos")),
      control_fetcher_(event_loop_->MakeFetcher<
                       frc971::control_loops::drivetrain::LocalizerControl>(
          "/drivetrain")) {
  if (dt_config_.is_simulated) {
    down_estimator_.assume_perfect_gravity();
  }

  for (size_t camera_index = 0; camera_index < kNumCameras; ++camera_index) {
    const std::string_view channel_name = kDetectionChannels.at(camera_index);
    const aos::Channel *const channel = CHECK_NOTNULL(
        event_loop->GetChannel<frc971::vision::TargetMap>(channel_name));
    event_loop->MakeWatcher(
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
                    RejectionReason::IMAGE_FROM_FUTURE);
            return;
          }
          auto debug_builder =
              cameras_.at(camera_index).debug_sender.MakeStaticBuilder();
          auto target_debug_list = debug_builder->add_targets();
          // The static_length should already be 20.
          CHECK(target_debug_list->reserve(20));
          for (const frc971::vision::TargetPoseFbs *target :
               *targets.target_poses()) {
            VLOG(1) << "Handling target from " << camera_index;
            HandleTarget(camera_index, orin_capture_time, *target,
                         target_debug_list->emplace_back());
          }
          StatisticsForCamera(cameras_.at(camera_index),
                              debug_builder->add_statistics());
          debug_builder.CheckOk(debug_builder.Send());
          SendStatus();
        });
  }

  event_loop_->AddPhasedLoop([this](int) { SendOutput(); },
                             std::chrono::milliseconds(20));

  event_loop_->MakeWatcher(
      "/drivetrain",
      [this](
          const frc971::control_loops::drivetrain::LocalizerControl &control) {
        HandleControl(control);
      });

  ekf_.set_ignore_accel(true);
  // Priority should be lower than the imu reading process, but non-zero.
  event_loop->SetRuntimeRealtimePriority(10);
  event_loop->OnRun([this, event_loop]() {
    ekf_.ResetInitialState(event_loop->monotonic_now(),
                           HybridEkf::State::Zero(), NominalCovariance());
    if (control_fetcher_.Fetch()) {
      HandleControl(*control_fetcher_.get());
    }
  });
}

void Localizer::HandleControl(
    const frc971::control_loops::drivetrain::LocalizerControl &control) {
  // This is triggered whenever we need to force the X/Y/(maybe theta)
  // position of the robot to a particular point---e.g., during pre-match
  // setup, or when commanded by a button on the driverstation.

  // For some forms of reset, we choose to keep our current yaw estimate
  // rather than overriding it from the control message.
  const double theta = control.keep_current_theta()
                           ? ekf_.X_hat(StateIdx::kTheta)
                           : control.theta();
  // Encoder values need to be reset based on the current values to ensure
  // that we don't get weird corrections on the next encoder update.
  const double left_encoder = ekf_.X_hat(StateIdx::kLeftEncoder);
  const double right_encoder = ekf_.X_hat(StateIdx::kRightEncoder);
  ekf_.ResetInitialState(t_,
                         (HybridEkf::State() << control.x(), control.y(), theta,
                          left_encoder, 0, right_encoder, 0, 0, 0, 0, 0, 0)
                             .finished(),
                         NominalCovariance());
  VLOG(1) << "Reset state";
}

void Localizer::HandleImu(aos::monotonic_clock::time_point /*sample_time_pico*/,
                          aos::monotonic_clock::time_point sample_time_orin,
                          std::optional<Eigen::Vector2d> /*encoders*/,
                          Eigen::Vector3d gyro, Eigen::Vector3d accel) {
  std::optional<Eigen::Vector2d> encoders = utils_.Encoders(sample_time_orin);
  last_encoder_readings_ = encoders;
  VLOG(1) << "Got encoders";
  if (t_ == aos::monotonic_clock::min_time) {
    t_ = sample_time_orin;
  }
  if (t_ + 10 * frc971::controls::ImuWatcher::kNominalDt < sample_time_orin) {
    t_ = sample_time_orin;
    ++clock_resets_;
  }
  const aos::monotonic_clock::duration dt = sample_time_orin - t_;
  t_ = sample_time_orin;
  // We don't actually use the down estimator currently, but it's really
  // convenient for debugging.
  down_estimator_.Predict(gyro, accel, dt);
  const double yaw_rate = (dt_config_.imu_transform * gyro)(2);
  ekf_.UpdateEncodersAndGyro(
      encoders.has_value() ? std::make_optional<double>(encoders.value()(0))
                           : std::nullopt,
      encoders.has_value() ? std::make_optional<double>(encoders.value()(1))
                           : std::nullopt,
      yaw_rate, utils_.VoltageOrZero(sample_time_orin), accel, t_);
  SendStatus();
}

void Localizer::RejectImage(int camera_index, RejectionReason reason,
                            TargetEstimateDebugStatic *builder) {
  if (builder != nullptr) {
    builder->set_accepted(false);
    builder->set_rejection_reason(reason);
  }
  cameras_.at(camera_index).rejection_counter.IncrementError(reason);
}

// Only use april tags present in the target map; this method has also been used
// (in the past) for ignoring april tags that tend to produce problematic
// readings.
bool Localizer::UseAprilTag(uint64_t target_id) {
  if (target_poses_.count(target_id) == 0) {
    return false;
  }
  return true;
}

bool Localizer::DeweightAprilTag(uint64_t target_id) {
  const flatbuffers::Vector<uint64_t> *ignore_tags = nullptr;

  switch (utils_.Alliance()) {
    case aos::Alliance::kRed:
      ignore_tags = CHECK_NOTNULL(
          constants_fetcher_.constants().common()->ignore_targets()->red());
      break;
    case aos::Alliance::kBlue:
      ignore_tags = CHECK_NOTNULL(
          constants_fetcher_.constants().common()->ignore_targets()->blue());
      break;
    case aos::Alliance::kInvalid:
      return false;
  }
  return std::find(ignore_tags->begin(), ignore_tags->end(), target_id) !=
         ignore_tags->end();
}

namespace {
// Converts a camera transformation matrix from treating the +Z axis from
// pointing straight out the lens to having the +X pointing straight out the
// lens, with +Z going "up" (i.e., -Y in the normal convention) and +Y going
// leftwards (i.e., -X in the normal convention).
Localizer::Transform ZToXCamera(const Localizer::Transform &transform) {
  return transform *
         Eigen::Matrix4d{
             {0, -1, 0, 0}, {0, 0, -1, 0}, {1, 0, 0, 0}, {0, 0, 0, 1}};
}
}  // namespace

void Localizer::HandleTarget(
    int camera_index, const aos::monotonic_clock::time_point capture_time,
    const frc971::vision::TargetPoseFbs &target,
    TargetEstimateDebugStatic *debug_builder) {
  ++total_candidate_targets_;
  ++cameras_.at(camera_index).total_candidate_targets;
  const uint64_t target_id = target.id();

  if (debug_builder == nullptr) {
    AOS_LOG(ERROR, "Dropped message from debug vector.");
  } else {
    debug_builder->set_camera(camera_index);
    debug_builder->set_image_age_sec(aos::time::DurationInSeconds(
        event_loop_->monotonic_now() - capture_time));
    debug_builder->set_image_monotonic_timestamp_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            capture_time.time_since_epoch())
            .count());
    debug_builder->set_april_tag(target_id);
  }
  VLOG(2) << aos::FlatbufferToJson(&target);
  if (!UseAprilTag(target_id)) {
    VLOG(1) << "Rejecting target due to invalid ID " << target_id;
    RejectImage(camera_index, RejectionReason::NO_SUCH_TARGET, debug_builder);
    return;
  }
  double april_tag_noise_scalar = 1.0;
  if (DeweightAprilTag(target_id)) {
    if (!FLAGS_always_use_extra_tags && utils_.MaybeInAutonomous()) {
      VLOG(1) << "Rejecting target due to auto invalid ID " << target_id;
      RejectImage(camera_index, RejectionReason::NO_SUCH_TARGET, debug_builder);
      return;
    } else {
      april_tag_noise_scalar = 10.0;
    }
  }

  const Transform &H_field_target = target_poses_.at(target_id);
  const Transform &H_robot_camera = cameras_.at(camera_index).extrinsics;

  const Transform H_camera_target = PoseToTransform(&target);

  // In order to do the EKF correction, we determine the expected state based
  // on the state at the time the image was captured; however, we insert the
  // correction update itself at the current time. This is technically not
  // quite correct, but saves substantial CPU usage & code complexity by
  // making it so that we don't have to constantly rewind the entire EKF
  // history.
  const std::optional<State> state_at_capture =
      ekf_.LastStateBeforeTime(capture_time);

  if (!state_at_capture.has_value()) {
    VLOG(1) << "Rejecting image due to being too old.";
    return RejectImage(camera_index, RejectionReason::IMAGE_TOO_OLD,
                       debug_builder);
  } else if (target.pose_error() > FLAGS_max_pose_error) {
    VLOG(1) << "Rejecting target due to high pose error "
            << target.pose_error();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR,
                       debug_builder);
  } else if (target.pose_error_ratio() > FLAGS_max_pose_error_ratio) {
    VLOG(1) << "Rejecting target due to high pose error ratio "
            << target.pose_error_ratio();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR_RATIO,
                       debug_builder);
  }

  Corrector corrector(state_at_capture.value(), H_field_target, H_robot_camera,
                      H_camera_target);
  const double distance_to_target = corrector.observed()(Corrector::kDistance);

  // Heading, distance, skew at 1 meter.
  Eigen::Matrix<double, 3, 1> noises(0.01, 0.05, 0.05);
  const double distance_noise_scalar =
      std::min(1.0, std::pow(distance_to_target, 2.0));
  noises(Corrector::kDistance) *= distance_noise_scalar;
  noises(Corrector::kSkew) *= distance_noise_scalar;
  // TODO(james): This is leftover from last year; figure out if we want it.
  // Scale noise by the distortion factor for this detection
  noises *= (1.0 + FLAGS_distortion_noise_scalar * target.distortion_factor());
  noises *= april_tag_noise_scalar;

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R.diagonal() = noises.cwiseAbs2();
  const Eigen::Vector3d camera_position =
      corrector.observed_camera_pose().abs_pos();
  // Calculate the camera-to-robot transformation matrix ignoring the
  // pitch/roll of the camera.
  const Transform H_camera_robot_stripped =
      frc971::control_loops::Pose(ZToXCamera(H_robot_camera))
          .AsTransformationMatrix()
          .inverse();
  const frc971::control_loops::Pose measured_pose(
      corrector.observed_camera_pose().AsTransformationMatrix() *
      H_camera_robot_stripped);
  if (debug_builder != nullptr) {
    debug_builder->set_camera_x(camera_position.x());
    debug_builder->set_camera_y(camera_position.y());
    debug_builder->set_camera_theta(
        corrector.observed_camera_pose().abs_theta());
    debug_builder->set_implied_robot_x(measured_pose.rel_pos().x());
    debug_builder->set_implied_robot_y(measured_pose.rel_pos().y());
    debug_builder->set_implied_robot_theta(measured_pose.rel_theta());

    Corrector::PopulateMeasurement(corrector.expected(),
                                   debug_builder->add_expected_observation());
    Corrector::PopulateMeasurement(corrector.observed(),
                                   debug_builder->add_actual_observation());
    Corrector::PopulateMeasurement(noises, debug_builder->add_modeled_noise());
  }

  const double camera_yaw_error =
      aos::math::NormalizeAngle(corrector.expected_camera_pose().abs_theta() -
                                corrector.observed_camera_pose().abs_theta());
  constexpr double kDegToRad = M_PI / 180.0;

  const double robot_speed =
      (state_at_capture.value()(StateIdx::kLeftVelocity) +
       state_at_capture.value()(StateIdx::kRightVelocity)) /
      2.0;
  const double yaw_threshold =
      (utils_.MaybeInAutonomous() ? FLAGS_max_implied_yaw_error
                                  : FLAGS_max_implied_teleop_yaw_error) *
      kDegToRad;

  if (target.distortion_factor() > FLAGS_max_distortion) {
    VLOG(1) << "Rejecting target due to high distortion.";
    return RejectImage(camera_index, RejectionReason::HIGH_DISTORTION,
                       debug_builder);
  } else if (utils_.MaybeInAutonomous() &&
             (std::abs(robot_speed) > FLAGS_max_auto_image_robot_speed)) {
    return RejectImage(camera_index, RejectionReason::ROBOT_TOO_FAST,
                       debug_builder);
  } else if (std::abs(camera_yaw_error) > yaw_threshold) {
    return RejectImage(camera_index, RejectionReason::HIGH_IMPLIED_YAW_ERROR,
                       debug_builder);
  } else if (distance_to_target > FLAGS_max_distance_to_target) {
    return RejectImage(camera_index, RejectionReason::HIGH_DISTANCE_TO_TARGET,
                       debug_builder);
  }

  const Input U = ekf_.MostRecentInput();
  VLOG(1) << "previous state " << ekf_.X_hat().transpose();
  const State prior_state = ekf_.X_hat();
  // For the correction step, instead of passing in the measurement directly,
  // we pass in (0, 0, 0) as the measurement and then for the expected
  // measurement (Zhat) we calculate the error between the pose implied by
  // the camera measurement and the current estimate of the
  // pose. This doesn't affect any of the math, it just makes the code a bit
  // more convenient to write given the Correct() interface we already have.
  if (FLAGS_do_xytheta_corrections) {
    Eigen::Vector3d Z(measured_pose.rel_pos().x(), measured_pose.rel_pos().y(),
                      measured_pose.rel_theta());
    Eigen::Matrix<double, 3, 1> xyz_noises(0.2, 0.2, 0.5);
    xyz_noises *= distance_noise_scalar;
    xyz_noises *= april_tag_noise_scalar;
    // Scale noise by the distortion factor for this detection
    xyz_noises *=
        (1.0 + FLAGS_distortion_noise_scalar * target.distortion_factor());

    Eigen::Matrix3d R_xyz = Eigen::Matrix3d::Zero();
    R_xyz.diagonal() = xyz_noises.cwiseAbs2();
    xyz_observations_.CorrectKnownH(Eigen::Vector3d::Zero(), &U,
                                    XyzCorrector(state_at_capture.value(), Z),
                                    R_xyz, t_);
  } else {
    observations_.CorrectKnownH(Eigen::Vector3d::Zero(), &U, corrector, R, t_);
  }
  ++total_accepted_targets_;
  ++cameras_.at(camera_index).total_accepted_targets;
  VLOG(1) << "new state " << ekf_.X_hat().transpose();
  if (debug_builder != nullptr) {
    debug_builder->set_correction_x(ekf_.X_hat()(StateIdx::kX) -
                                    prior_state(StateIdx::kX));
    debug_builder->set_correction_y(ekf_.X_hat()(StateIdx::kY) -
                                    prior_state(StateIdx::kY));
    debug_builder->set_correction_theta(ekf_.X_hat()(StateIdx::kTheta) -
                                        prior_state(StateIdx::kTheta));
    debug_builder->set_accepted(true);
    debug_builder->set_expected_robot_x(ekf_.X_hat()(StateIdx::kX));
    debug_builder->set_expected_robot_y(ekf_.X_hat()(StateIdx::kY));
    debug_builder->set_expected_robot_theta(ekf_.X_hat()(StateIdx::kTheta));
  }
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
  server_statistics_fetcher_.Fetch();
  client_statistics_fetcher_.Fetch();

  bool orins_connected = true;

  if (server_statistics_fetcher_.get()) {
    for (const auto *orin_server_status :
         *server_statistics_fetcher_->connections()) {
      if (orin_server_status->state() ==
          aos::message_bridge::State::DISCONNECTED) {
        orins_connected = false;
      }
    }
  }

  if (client_statistics_fetcher_.get()) {
    for (const auto *pi_client_status :
         *client_statistics_fetcher_->connections()) {
      if (pi_client_status->state() ==
          aos::message_bridge::State::DISCONNECTED) {
        orins_connected = false;
      }
    }
  }

  // The output message is year-agnostic, and retains "pi" naming for histrocial
  // reasons.
  output_builder.add_all_pis_connected(orins_connected);
  builder.CheckOk(builder.Send(output_builder.Finish()));
}

flatbuffers::Offset<frc971::control_loops::drivetrain::LocalizerState>
Localizer::PopulateState(const State &X_hat,
                         flatbuffers::FlatBufferBuilder *fbb) {
  frc971::control_loops::drivetrain::LocalizerState::Builder builder(*fbb);
  builder.add_x(X_hat(StateIdx::kX));
  builder.add_y(X_hat(StateIdx::kY));
  builder.add_theta(aos::math::NormalizeAngle(X_hat(StateIdx::kTheta)));
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
    builder.add_board_offset_ns(imu_watcher_.pico_offset().value().count());
    builder.add_board_offset_error_ns(imu_watcher_.pico_offset_error().count());
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

void Localizer::StatisticsForCamera(const CameraState &camera,
                                    CumulativeStatisticsStatic *builder) {
  camera.rejection_counter.PopulateCountsStaticFbs(
      builder->add_rejection_reasons());
  builder->set_total_accepted(camera.total_accepted_targets);
  builder->set_total_candidates(camera.total_candidate_targets);
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
  // covariance is a square; we use the number of rows in the state as the rows
  // and cols of the covariance.
  auto covariance_offset =
      frc971::FromEigen<State::RowsAtCompileTime, State::RowsAtCompileTime>(
          ekf_.P(), builder.fbb());
  Status::Builder status_builder = builder.MakeBuilder<Status>();
  status_builder.add_state(state_offset);
  status_builder.add_down_estimator(down_estimator_offset);
  status_builder.add_imu(imu_offset);
  status_builder.add_statistics(stats_offset);
  status_builder.add_ekf_covariance(covariance_offset);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

Eigen::Vector3d Localizer::Corrector::HeadingDistanceSkew(
    const Pose &relative_pose) {
  const double heading = relative_pose.heading();
  const double distance = relative_pose.xy_norm();
  const double skew =
      ::aos::math::NormalizeAngle(relative_pose.rel_theta() - heading);
  return {heading, distance, skew};
}

Localizer::Corrector Localizer::Corrector::CalculateHeadingDistanceSkewH(
    const State &state_at_capture, const Transform &H_field_target,
    const Transform &H_robot_camera, const Transform &H_camera_target) {
  const Transform H_field_camera = H_field_target * H_camera_target.inverse();
  const Pose expected_robot_pose(
      {state_at_capture(StateIdx::kX), state_at_capture(StateIdx::kY), 0.0},
      state_at_capture(StateIdx::kTheta));
  // Observed position on the field, reduced to just the 2-D pose.
  const Pose observed_camera(ZToXCamera(H_field_camera));
  const Pose expected_camera(expected_robot_pose.AsTransformationMatrix() *
                             ZToXCamera(H_robot_camera));
  const Pose nominal_target(ZToXCamera(H_field_target));
  const Pose observed_target = nominal_target.Rebase(&observed_camera);
  const Pose expected_target = nominal_target.Rebase(&expected_camera);
  return Localizer::Corrector{
      expected_robot_pose,
      observed_camera,
      expected_camera,
      HeadingDistanceSkew(expected_target),
      HeadingDistanceSkew(observed_target),
      frc971::control_loops::drivetrain::HMatrixForCameraHeadingDistanceSkew(
          nominal_target, observed_camera)};
}

Localizer::Corrector::Corrector(const State &state_at_capture,
                                const Transform &H_field_target,
                                const Transform &H_robot_camera,
                                const Transform &H_camera_target)
    : Corrector(CalculateHeadingDistanceSkewH(
          state_at_capture, H_field_target, H_robot_camera, H_camera_target)) {}

Localizer::Output Localizer::Corrector::H(const State &, const Input &) {
  return expected_ - observed_;
}

Localizer::Output Localizer::XyzCorrector::H(const State &, const Input &) {
  CHECK(Z_.allFinite());
  Eigen::Vector3d Zhat = H_ * state_at_capture_ - Z_;
  // Rewrap angle difference to put it back in range.
  Zhat(2) = aos::math::NormalizeAngle(Zhat(2));
  VLOG(1) << "Zhat " << Zhat.transpose() << " Z_ " << Z_.transpose()
          << " state " << (H_ * state_at_capture_).transpose();
  return Zhat;
}

}  // namespace y2024::localizer
