#include "frc971/vision/target_mapper.h"

#include <random>

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/random_seed.h"
#include "aos/util/math.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

DECLARE_int32(min_target_id);
DECLARE_int32(max_target_id);

namespace frc971::vision {

namespace {
constexpr double kToleranceMeters = 0.05;
constexpr double kToleranceRadians = 0.05;
// Conversions between euler angles and quaternion result in slightly-off
// doubles
constexpr double kOrientationEqTolerance = 1e-10;
constexpr std::chrono::milliseconds kMaxDt = std::chrono::milliseconds(3);
constexpr std::string_view kFieldName = "test";
}  // namespace

// Angles normalized by aos::math::NormalizeAngle()
#define EXPECT_NORMALIZED_ANGLES_NEAR(theta1, theta2, tolerance)           \
  {                                                                        \
    double delta = std::abs(aos::math::DiffAngle(theta1, theta2));         \
    /* Have to check delta - 2pi for the case that one angle is very */    \
    /* close to -pi, and the other is very close to +pi */                 \
    EXPECT_TRUE(delta < tolerance || std::abs(aos::math::DiffAngle(        \
                                         delta, 2.0 * M_PI)) < tolerance); \
  }

#define EXPECT_POSE_NEAR(pose1, pose2)                                     \
  {                                                                        \
    for (size_t i = 0; i < 3; i++) {                                       \
      EXPECT_NEAR(pose1.p(i), pose2.p(i), kToleranceMeters);               \
    }                                                                      \
    auto rpy_1 = PoseUtils::QuaternionToEulerAngles(pose1.q);              \
    auto rpy_2 = PoseUtils::QuaternionToEulerAngles(pose2.q);              \
    for (size_t i = 0; i < 3; i++) {                                       \
      SCOPED_TRACE(absl::StrFormat("rpy_1(%d) = %f, rpy_2(%d) = %f", i,    \
                                   rpy_1(i), i, rpy_2(i)));                \
      EXPECT_NORMALIZED_ANGLES_NEAR(rpy_1(i), rpy_2(i), kToleranceRadians) \
    }                                                                      \
  }

#define EXPECT_POSE_EQ(pose1, pose2) \
  EXPECT_EQ(pose1.p, pose2.p);       \
  EXPECT_EQ(pose1.q, pose2.q);

#define EXPECT_QUATERNION_NEAR(q1, q2)                                        \
  EXPECT_NEAR(q1.x(), q2.x(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.y(), q2.y(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.z(), q2.z(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.w(), q2.w(), kOrientationEqTolerance) << q1 << " != " << q2;

// Expects same roll, pitch, and yaw values (not equivalent rotation)
#define EXPECT_RPY_EQ(rpy_1, rpy_2)                                     \
  {                                                                     \
    for (size_t i = 0; i < 3; i++) {                                    \
      SCOPED_TRACE(absl::StrFormat("rpy_1(%d) = %f, rpy_2(%d) = %f", i, \
                                   rpy_1(i), i, rpy_2(i)));             \
      EXPECT_NORMALIZED_ANGLES_NEAR(rpy_1(i), rpy_2(i),                 \
                                    kOrientationEqTolerance)            \
    }                                                                   \
  }

#define EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(roll, pitch, yaw) \
  {                                                                        \
    auto rpy = Eigen::Vector3d(roll, pitch, yaw);                          \
    auto converted_rpy = PoseUtils::QuaternionToEulerAngles(               \
        PoseUtils::EulerAnglesToQuaternion(rpy));                          \
    EXPECT_RPY_EQ(converted_rpy, rpy);                                     \
  }

// Both confidence matrixes should have the same dimensions and be square
#define EXPECT_CONFIDENCE_GT(confidence1, confidence2) \
  {                                                    \
    ASSERT_EQ(confidence1.rows(), confidence2.rows()); \
    ASSERT_EQ(confidence1.rows(), confidence1.cols()); \
    ASSERT_EQ(confidence2.rows(), confidence2.cols()); \
    for (size_t i = 0; i < confidence1.rows(); i++) {  \
      EXPECT_GT(confidence1(i, i), confidence2(i, i)); \
    }                                                  \
  }

namespace {
ceres::examples::Pose3d Make2dPose(double x, double y, double yaw_radians) {
  return ceres::examples::Pose3d{Eigen::Vector3d(x, y, 0.0),
                                 PoseUtils::EulerAnglesToQuaternion(
                                     Eigen::Vector3d(0.0, 0.0, yaw_radians))};
}

// Assumes camera and robot origin are the same
DataAdapter::TimestampedDetection MakeTimestampedDetection(
    aos::distributed_clock::time_point time, Eigen::Affine3d H_robot_target,
    TargetMapper::TargetId id, double distortion_factor = 0.001) {
  auto target_pose = PoseUtils::Affine3dToPose3d(H_robot_target);
  return DataAdapter::TimestampedDetection{
      .time = time,
      .H_robot_target = H_robot_target,
      .distance_from_camera = target_pose.p.norm(),
      .distortion_factor = distortion_factor,
      .id = id};
}

DataAdapter::TimestampedDetection MakeTimestampedDetectionForConfidence(
    aos::distributed_clock::time_point time, TargetMapper::TargetId id,
    double distance_from_camera, double distortion_factor) {
  auto target_pose = ceres::examples::Pose3d{
      .p = Eigen::Vector3d(distance_from_camera, 0.0, 0.0),
      .q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};
  return DataAdapter::TimestampedDetection{
      .time = time,
      .H_robot_target = PoseUtils::Pose3dToAffine3d(target_pose),
      .distance_from_camera = target_pose.p.norm(),
      .distortion_factor = distortion_factor,
      .id = id};
}

bool TargetIsInView(TargetMapper::TargetPose target_detection) {
  // And check if it is within the fov of the robot /
  // camera, assuming camera is pointing in the
  // positive x-direction of the robot
  double angle_to_target =
      atan2(target_detection.pose.p(1), target_detection.pose.p(0));

  // Simulated camera field of view, in radians
  constexpr double kCameraFov = M_PI_2;
  if (std::abs(angle_to_target) <= kCameraFov / 2.0) {
    VLOG(2) << "Found target in view, based on T = "
            << target_detection.pose.p(0) << ", " << target_detection.pose.p(1)
            << " with angle " << angle_to_target;
    return true;
  } else {
    return false;
  }
}

aos::distributed_clock::time_point TimeInMs(size_t ms) {
  return aos::distributed_clock::time_point(std::chrono::milliseconds(ms));
}

}  // namespace

TEST(PoseUtilsTest, EulerAnglesAndQuaternionConversions) {
  // Make sure that the conversions are consistent back and forth.
  // These angles shouldn't get changed to a different, equivalent roll pitch
  // yaw.
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, M_PI);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, -M_PI);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, M_PI_2);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, -M_PI_2);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, 0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, M_PI_4, 0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, -M_PI_4, 0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, -M_PI_4, M_PI_4);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(M_PI_4, -M_PI_4, M_PI_4);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(-M_PI_2, -M_PI_4, M_PI_4);

  // Now, do a sweep of roll, pitch, and yaws in the normalized
  // range.
  // - roll: (-pi/2, pi/2)
  // - pitch: (-pi/2, pi/2)
  // - yaw: [-pi, pi)
  constexpr double kThetaMaxRoll = M_PI_2 - kToleranceRadians;
  constexpr double kThetaMaxPitch = M_PI_2 - kToleranceRadians;
  constexpr double kThetaMaxYaw = M_PI;
  constexpr double kDeltaTheta = M_PI / 16;

  for (double roll = -kThetaMaxRoll; roll < kThetaMaxRoll;
       roll += kDeltaTheta) {
    for (double pitch = -kThetaMaxPitch; pitch < kThetaMaxPitch;
         pitch += kDeltaTheta) {
      for (double yaw = -kThetaMaxYaw; yaw < kThetaMaxYaw; yaw += kDeltaTheta) {
        SCOPED_TRACE(
            absl::StrFormat("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw));
        EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(roll, pitch, yaw);
      }
    }
  }
}

TEST(DataAdapterTest, ComputeConfidence) {
  // Check the confidence matrices. Don't check the actual values
  // in case the constants change, just check that the confidence of contraints
  // decreases as time period or distances from camera increase.

  constexpr size_t kIdStart = 0;
  constexpr size_t kIdEnd = 1;

  {
    // Vary time period
    constexpr double kDistanceStart = 0.5;
    constexpr double kDistanceEnd = 2.0;
    constexpr double kDistortionFactor = 0.001;

    TargetMapper::ConfidenceMatrix last_confidence =
        TargetMapper::ConfidenceMatrix::Zero();
    for (size_t dt = 0; dt < 15; dt++) {
      TargetMapper::ConfidenceMatrix confidence =
          DataAdapter::ComputeConfidence(
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(0), kIdStart, kDistanceStart, kDistortionFactor),
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(dt), kIdEnd, kDistanceEnd, kDistortionFactor));

      if (dt != 0) {
        // Confidence only decreases every 5ms (control loop period)
        if (dt % 5 == 0) {
          EXPECT_CONFIDENCE_GT(last_confidence, confidence);
        } else {
          EXPECT_EQ(last_confidence, confidence);
        }
      }
      last_confidence = confidence;
    }
  }

  {
    // Vary distance at start
    constexpr int kDt = 3;
    constexpr double kDistanceEnd = 1.5;
    constexpr double kDistortionFactor = 0.001;

    TargetMapper::ConfidenceMatrix last_confidence =
        TargetMapper::ConfidenceMatrix::Zero();
    for (double distance_start = 0.0; distance_start < 3.0;
         distance_start += 0.5) {
      TargetMapper::ConfidenceMatrix confidence =
          DataAdapter::ComputeConfidence(
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(0), kIdStart, distance_start, kDistortionFactor),
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(kDt), kIdEnd, kDistanceEnd, kDistortionFactor));

      if (distance_start != 0.0) {
        EXPECT_CONFIDENCE_GT(last_confidence, confidence);
      }
      last_confidence = confidence;
    }
  }

  {
    // Vary distance at end
    constexpr int kDt = 2;
    constexpr double kDistanceStart = 2.5;
    constexpr double kDistortionFactor = 0.001;

    TargetMapper::ConfidenceMatrix last_confidence =
        TargetMapper::ConfidenceMatrix::Zero();
    for (double distance_end = 0.0; distance_end < 3.0; distance_end += 0.5) {
      TargetMapper::ConfidenceMatrix confidence =
          DataAdapter::ComputeConfidence(
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(0), kIdStart, kDistanceStart, kDistortionFactor),
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(kDt), kIdEnd, distance_end, kDistortionFactor));

      if (distance_end != 0.0) {
        EXPECT_CONFIDENCE_GT(last_confidence, confidence);
      }
      last_confidence = confidence;
    }
  }

  {
    // Vary distortion factor
    constexpr int kDt = 2;
    constexpr double kDistanceStart = 2.5;
    constexpr double kDistanceEnd = 1.5;

    TargetMapper::ConfidenceMatrix last_confidence =
        TargetMapper::ConfidenceMatrix::Zero();
    for (double distortion_factor = 0.0; distortion_factor <= 1.0;
         distortion_factor += 0.01) {
      TargetMapper::ConfidenceMatrix confidence =
          DataAdapter::ComputeConfidence(
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(0), kIdStart, kDistanceStart, distortion_factor),
              MakeTimestampedDetectionForConfidence(
                  TimeInMs(kDt), kIdEnd, kDistanceEnd, distortion_factor));

      if (distortion_factor != 0.0) {
        EXPECT_CONFIDENCE_GT(last_confidence, confidence);
      }
      last_confidence = confidence;
    }
  }
}

TEST(DataAdapterTest, MatchTargetDetections) {
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {MakeTimestampedDetection(
           TimeInMs(5), PoseUtils::Pose3dToAffine3d(Make2dPose(5.0, -5.0, 0.0)),
           2),
       MakeTimestampedDetection(
           TimeInMs(6),
           PoseUtils::Pose3dToAffine3d(Make2dPose(5.0, -4.0, M_PI)), 0),
       MakeTimestampedDetection(
           TimeInMs(10),
           PoseUtils::Pose3dToAffine3d(Make2dPose(3.0, -3.0, M_PI)), 1),
       MakeTimestampedDetection(
           TimeInMs(13),
           PoseUtils::Pose3dToAffine3d(Make2dPose(4.0, -7.0, M_PI_2)), 2),
       MakeTimestampedDetection(
           TimeInMs(14),
           PoseUtils::Pose3dToAffine3d(Make2dPose(4.0, -4.0, M_PI_2)), 2)};

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);

  // The constraint between the detection at 6ms and the one at 10 ms is skipped
  // because dt > kMaxDt.
  // Also, the constraint between the last two detections is skipped because
  // they are the same target
  EXPECT_EQ(target_constraints.size(),
            timestamped_target_detections.size() - 3);

  // Between 5ms and 6ms detections
  EXPECT_DOUBLE_EQ(target_constraints[0].t_be.p(0), 0.0);
  EXPECT_DOUBLE_EQ(target_constraints[0].t_be.p(1), 1.0);
  EXPECT_QUATERNION_NEAR(
      target_constraints[0].t_be.q,
      PoseUtils::EulerAnglesToQuaternion(Eigen::Vector3d(0.0, 0.0, M_PI)));
  EXPECT_EQ(target_constraints[0].id_begin, 2);
  EXPECT_EQ(target_constraints[0].id_end, 0);

  // Between 10ms and 13ms detections
  EXPECT_DOUBLE_EQ(target_constraints[1].t_be.p(0), -1.0);
  EXPECT_DOUBLE_EQ(target_constraints[1].t_be.p(1), 4.0);
  EXPECT_QUATERNION_NEAR(
      target_constraints[1].t_be.q,
      PoseUtils::EulerAnglesToQuaternion(Eigen::Vector3d(0.0, 0.0, -M_PI_2)));
  EXPECT_EQ(target_constraints[1].id_begin, 1);
  EXPECT_EQ(target_constraints[1].id_end, 2);
}

TEST(TargetMapperTest, TwoTargetsOneConstraint) {
  FLAGS_min_target_id = 0;
  FLAGS_max_target_id = 1;

  ceres::examples::MapOfPoses target_poses;
  target_poses[0] = Make2dPose(5.0, 0.0, M_PI);
  target_poses[1] = Make2dPose(-5.0, 0.0, 0.0);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {MakeTimestampedDetection(
           TimeInMs(5), PoseUtils::Pose3dToAffine3d(Make2dPose(3.0, 0.0, M_PI)),
           0),
       MakeTimestampedDetection(
           TimeInMs(6), PoseUtils::Pose3dToAffine3d(Make2dPose(-7.0, 0.0, 0.0)),
           1)};
  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], Make2dPose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], Make2dPose(-5.0, 0.0, 0.0));
}

TEST(TargetMapperTest, TwoTargetsTwoConstraints) {
  FLAGS_min_target_id = 0;
  FLAGS_max_target_id = 1;

  ceres::examples::MapOfPoses target_poses;
  target_poses[0] = Make2dPose(5.0, 0.0, M_PI);
  target_poses[1] = Make2dPose(-5.0, 0.0, -M_PI_2);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {MakeTimestampedDetection(
           TimeInMs(5),
           PoseUtils::Pose3dToAffine3d(Make2dPose(6.0, 0.0, M_PI_2)), 0),
       MakeTimestampedDetection(
           TimeInMs(7),
           PoseUtils::Pose3dToAffine3d(Make2dPose(6.0, 10.0, -M_PI)), 1),
       MakeTimestampedDetection(
           TimeInMs(12),
           PoseUtils::Pose3dToAffine3d(Make2dPose(1.0, 0.0, M_PI)), 0),
       MakeTimestampedDetection(
           TimeInMs(13),
           PoseUtils::Pose3dToAffine3d(Make2dPose(-9.0, 0.0, -M_PI_2)), 1)};
  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], Make2dPose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], Make2dPose(-5.0, 0.0, -M_PI_2));
}

TEST(TargetMapperTest, TwoTargetsOneNoisyConstraint) {
  FLAGS_min_target_id = 0;
  FLAGS_max_target_id = 1;

  ceres::examples::MapOfPoses target_poses;
  target_poses[0] = Make2dPose(5.0, 0.0, M_PI);
  target_poses[1] = Make2dPose(-5.0, 0.0, 0.0);

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {MakeTimestampedDetection(
           TimeInMs(5),
           PoseUtils::Pose3dToAffine3d(Make2dPose(3.01, 0.001, M_PI - 0.001)),
           0),
       MakeTimestampedDetection(
           TimeInMs(7),
           PoseUtils::Pose3dToAffine3d(Make2dPose(-7.01, 0.0, 0.0)), 1)};

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], Make2dPose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], Make2dPose(-5.0, 0.0, 0.0));
}

class ChargedUpTargetMapperTest : public ::testing::Test {
 public:
  ChargedUpTargetMapperTest() : generator_(aos::testing::RandomSeed()) {}

  // Generates noisy target detection if target in camera FOV
  std::optional<DataAdapter::TimestampedDetection> MaybeGenerateNoisyDetection(
      const ceres::examples::Pose3d &robot_pose,
      const TargetMapper::TargetPose &target_pose, size_t t,
      bool force_in_view = false) {
    TargetMapper::TargetPose target_detection = {
        .id = target_pose.id,
        .pose = PoseUtils::ComputeRelativePose(robot_pose, target_pose.pose)};
    if (force_in_view || TargetIsInView(target_detection)) {
      // Define random generator with Gaussian
      // distribution
      constexpr double kMean = 0.0;
      constexpr double kStdDev = 1.0;
      // Can play with this to see how it impacts
      // randomness
      constexpr double kNoiseScalePosition = 0.01;
      constexpr double kNoiseScaleOrientation = 0.0005;
      std::normal_distribution<double> dist(kMean, kStdDev);

      target_detection.pose.p(0) += dist(generator_) * kNoiseScalePosition;
      target_detection.pose.p(1) += dist(generator_) * kNoiseScalePosition;
      target_detection.pose.p(2) += dist(generator_) * kNoiseScalePosition;

      target_detection.pose.q.w() += dist(generator_) * kNoiseScaleOrientation;
      target_detection.pose.q.x() += dist(generator_) * kNoiseScaleOrientation;
      target_detection.pose.q.y() += dist(generator_) * kNoiseScaleOrientation;
      target_detection.pose.q.z() += dist(generator_) * kNoiseScaleOrientation;

      // Get most distortion factors close to zero, but have a few outliers
      const std::vector<double> kDistortionFactorIntervals = {0.0, 0.01, 1.0};
      const std::vector<double> kDistortionFactorWeights = {0.9, 0.1};
      std::piecewise_constant_distribution<double> distortion_factor_dist(
          kDistortionFactorIntervals.begin(), kDistortionFactorIntervals.end(),
          kDistortionFactorWeights.begin());
      double distortion_factor = distortion_factor_dist(generator_);

      auto time_point = TimeInMs(t);
      return MakeTimestampedDetection(
          time_point, PoseUtils::Pose3dToAffine3d(target_detection.pose),
          target_detection.id, distortion_factor);
    }

    return std::nullopt;
  }

 private:
  std::default_random_engine generator_;
};

// Drive in a circle around the 2023 field, and add a bit of randomness to 3d
// pose detections
TEST_F(ChargedUpTargetMapperTest, FieldCircleMotion) {
  // Read target map
  auto target_map_fbs = aos::JsonFileToFlatbuffer<TargetMap>(
      aos::testing::ArtifactPath("frc971/vision/target_map.json"));

  std::vector<TargetMapper::TargetPose> actual_target_poses;
  ceres::examples::MapOfPoses target_poses;
  for (const auto *target_pose_fbs : *target_map_fbs.message().target_poses()) {
    const TargetMapper::TargetPose target_pose =
        PoseUtils::TargetPoseFromFbs(*target_pose_fbs);
    actual_target_poses.emplace_back(target_pose);
    target_poses[target_pose.id] = target_pose.pose;
  }

  double kFieldHalfLength = 16.54 / 2.0;  // half length of the field
  double kFieldHalfWidth = 8.02 / 2.0;    // half width of the field

  // Now, create a bunch of robot poses and target
  // observations
  constexpr size_t kDt = 5;
  constexpr double kRobotZ = 1.0;

  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  constexpr size_t kTotalSteps = 100;
  for (size_t step_count = 0; step_count < kTotalSteps; step_count++) {
    size_t t = kDt * step_count;

    // Circle clockwise around the center of the field
    double robot_theta = t;

    constexpr double kRadiusScalar = 0.5;
    double robot_x = (kRadiusScalar * kFieldHalfLength) * cos(robot_theta);
    double robot_y = (kRadiusScalar * -kFieldHalfWidth) * sin(robot_theta);

    auto robot_pose =
        ceres::examples::Pose3d{.p = Eigen::Vector3d(robot_x, robot_y, kRobotZ),
                                .q = PoseUtils::EulerAnglesToQuaternion(
                                    Eigen::Vector3d(robot_theta, 0.0, 0.0))};

    for (TargetMapper::TargetPose target_pose : actual_target_poses) {
      auto optional_detection =
          MaybeGenerateNoisyDetection(robot_pose, target_pose, t);
      if (optional_detection.has_value()) {
        timestamped_target_detections.emplace_back(*optional_detection);
      }
    }
  }

  // The above circular motion only captures targets 1-4, so add another
  // detection with the camera looking at targets 5-8, and 4 at the same time to
  // have a connection to the rest of the targets
  {
    auto last_robot_pose =
        ceres::examples::Pose3d{.p = Eigen::Vector3d(0.0, 0.0, kRobotZ),
                                .q = PoseUtils::EulerAnglesToQuaternion(
                                    Eigen::Vector3d(0.0, 0.0, M_PI))};
    for (size_t id = 4; id <= 8; id++) {
      auto target_pose =
          TargetMapper::GetTargetPoseById(actual_target_poses, id).value();
      auto optional_detection = MaybeGenerateNoisyDetection(
          last_robot_pose, target_pose, kTotalSteps * kDt, true);

      ASSERT_TRUE(optional_detection.has_value())
          << "Didn't detect target " << target_pose.id;
      timestamped_target_detections.emplace_back(*optional_detection);
    }
  }

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);
  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  for (auto [target_pose_id, mapper_target_pose] : mapper.target_poses()) {
    TargetMapper::TargetPose actual_target_pose =
        TargetMapper::GetTargetPoseById(actual_target_poses, target_pose_id)
            .value();
    EXPECT_POSE_NEAR(mapper_target_pose, actual_target_pose.pose);
  }
}

}  // namespace frc971::vision
