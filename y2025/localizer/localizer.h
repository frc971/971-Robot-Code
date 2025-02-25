#ifndef Y2025_LOCALIZER_LOCALIZER_H_
#define Y2025_LOCALIZER_LOCALIZER_H_

#include <map>

#include "aos/time/time.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/localization/utils.h"
#include "frc971/control_loops/swerve/autonomous_init_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/control_loops/swerve/swerve_localizer_state_static.h"
#include "frc971/estimation/kf.h"
#include "frc971/vision/target_map_generated.h"
#include "y2025/constants/constants_generated.h"
#include "y2025/localizer/status_static.h"

namespace y2025::localizer {

// This localizer is made to blend together the pose we get from our apriltag
// detection and the velocity estimation we do for our swerve LQR into one final
// pose x, y, theta. There are three main approaches we're choosing to take to
// do this because all of this code is new.
//
// 1. Weighted Average (Implemeneted)
//  In this approach we blend all of the pose estimation from each camera
//  together into a final vision-based pose by taking the weighted average of
//  each estimated pose where the weight is 1/(apriltag_distance_to_robot^2). We
//  then combine these with the position found by taking the integral of the
//  velocity in the naive estimator by taking a weighted average between the two
//  poses. The main advantages of this approach is its simplicity, making it
//  fairly easy to debug and change.
//
// 2. WPILib Swerve Pose Estimator (Unimplemented)
//  This approach is using the built-in WPIlib SwervePoseEstimator class to
//  blend together our encoders, gyro, and vision estimates into one final pose.
//  We have this in here in order to make sure we will have some baseline to use
//  and test against. The main advantage of this approach is its ease of use.
//
// 3. Kalman Filter (Implemented)
//  In this approach we use a Kalman filter to blend our naive estimator's
//  velocity readings with the vision pose's. We can use a simple Kalman Filter
//  for this approach because generally the system for blending the sensors can
//  be simplified to be fairly linear, unlike the overall swerve system. The
//  main advantages to this approach are its general robustness and relative
//  simplicity compared to more complicated sensor fusion.

class Localizer {
 public:
  typedef Eigen::Matrix<double, 4, 4> Transform;
  using Pose = frc971::control_loops::Pose;
  using LocalizerStateStatic =
      frc971::control_loops::swerve::LocalizerStateStatic;

  enum PoseIdx {
    kX = 0,
    kY = 1,
    kTheta = 2,
  };

  typedef Eigen::Matrix<double, 3, 1> XYThetaPose;

  static constexpr int kNumCameras = 4;

  struct DebugPoseEstimate {
    uint64_t camera;
    uint64_t april_tag;
    XYThetaPose robot_pose;
  };

  struct CameraState {
    Transform extrinsics = Transform::Zero();
    aos::util::ArrayErrorCounter<RejectionReason, RejectionCount>
        rejection_counter;
    size_t total_candidate_targets = 0;
    size_t total_accepted_targets = 0;
  };

  struct EstimatedSwerveState {
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
  };

  Localizer(aos::EventLoop *event_loop);

  void HandleTargetPoseFbs(const frc971::vision::TargetPoseFbs *target_pose,
                           size_t camera_index,
                           aos::monotonic_clock::time_point capture_time,
                           aos::monotonic_clock::time_point now);
  virtual void HandleDetectedRobotPose(
      XYThetaPose pose, double distance_to_robot, uint64_t target_id,
      double distortion_factor, aos::monotonic_clock::time_point capture_time,
      aos::monotonic_clock::time_point now) = 0;

  void HandleSwerveStatus(const frc971::control_loops::swerve::Status &status,
                          aos::monotonic_clock::time_point now);

  virtual void HandleEstimatedSwerveState(
      EstimatedSwerveState estimated_state,
      aos::monotonic_clock::time_point now) = 0;
  virtual void HandleAutonomousInit(
      const frc971::control_loops::swerve::AutonomousInit &init_message) = 0;
  virtual void SendOutput(
      aos::Sender<LocalizerStateStatic>::StaticBuilder state_builder) = 0;

  void SendStatus();

 protected:
  aos::EventLoop *event_loop_;
  XYThetaPose estimated_pose_ = XYThetaPose::Zero();

 private:
  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;

  aos::Sender<frc971::control_loops::swerve::LocalizerStateStatic>
      localizer_state_sender_;

  aos::Sender<StatusStatic> localizer_status_sender_;

  std::array<CameraState, kNumCameras> cameras_;
  const std::map<uint64_t, Transform> target_poses_;

  std::vector<DebugPoseEstimate> debug_estimates_;

  frc971::control_loops::drivetrain::LocalizationUtils utils_;

  std::optional<aos::monotonic_clock::time_point>
      last_estimated_velocity_timestamp = std::nullopt;
};

class WeightedAverageLocalizer : Localizer {
 public:
  static constexpr int kMaxPreviousSamples = 8;
  struct DetectionInfo {
    double distance_to_robot;
    aos::monotonic_clock::time_point time_detected;
    XYThetaPose pose = XYThetaPose::Zero();
  };

  struct WeighingDetectionInfo {
    double weighing_metric;
    XYThetaPose pose = XYThetaPose::Zero();
  };

  WeightedAverageLocalizer(aos::EventLoop *event_loop);

  void HandleDetectedRobotPose(XYThetaPose pose, double distance_to_robot,
                               uint64_t target_id, double distortion_factor,
                               aos::monotonic_clock::time_point capture_time,
                               aos::monotonic_clock::time_point now) override;

  void HandleEstimatedSwerveState(
      EstimatedSwerveState estimated_state,
      aos::monotonic_clock::time_point now) override;

  void HandleAutonomousInit(const frc971::control_loops::swerve::AutonomousInit
                                &init_message) override;

  void SendOutput(
      aos::Sender<LocalizerStateStatic>::StaticBuilder state_builder) override;

 private:
  std::map<uint64_t, DetectionInfo> detected_pose_;
  std::vector<XYThetaPose> previous_samples_;
  aos::monotonic_clock::time_point start_time_;
  bool first_it_ = true;
};

class KalmanFilterLocalizer : Localizer {
 public:
  static constexpr int kNumStates = 6;
  typedef frc971::estimation::Kf<kNumStates> KalmanFilter;

  enum StateIdx {
    kX = 0,
    kY = 1,
    kTheta = 2,
    kVx = 3,
    kVy = 4,
    kOmega = 5,
  };

  KalmanFilterLocalizer(aos::EventLoop *event_loop);

  void HandleDetectedRobotPose(XYThetaPose pose, double distance_to_robot,
                               uint64_t target_id, double distortion_factor,
                               aos::monotonic_clock::time_point capture_time,
                               aos::monotonic_clock::time_point now) override;

  void HandleEstimatedSwerveState(
      EstimatedSwerveState estimated_state,
      aos::monotonic_clock::time_point now) override;

  void HandleAutonomousInit(const frc971::control_loops::swerve::AutonomousInit
                                &init_message) override;

  void SendOutput(
      aos::Sender<LocalizerStateStatic>::StaticBuilder state_builder) override;

 private:
  KalmanFilter kalman_filter_;
};

}  // namespace y2025::localizer

#endif  // Y2025_LOCALIZER_LOCALIZER_H_
