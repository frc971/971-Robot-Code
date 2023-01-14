#include "frc971/vision/target_mapper.h"

#include <random>

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/random_seed.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace frc971::vision {

namespace {
constexpr double kToleranceMeters = 0.05;
constexpr double kToleranceRadians = 0.05;
constexpr std::string_view kFieldName = "test";
}  // namespace

#define EXPECT_POSE_NEAR(pose1, pose2)             \
  EXPECT_NEAR(pose1.x, pose2.x, kToleranceMeters); \
  EXPECT_NEAR(pose1.y, pose2.y, kToleranceMeters); \
  EXPECT_NEAR(pose1.yaw_radians, pose2.yaw_radians, kToleranceRadians);

#define EXPECT_POSE_EQ(pose1, pose2)  \
  EXPECT_DOUBLE_EQ(pose1.x, pose2.x); \
  EXPECT_DOUBLE_EQ(pose1.y, pose2.y); \
  EXPECT_DOUBLE_EQ(pose1.yaw_radians, pose2.yaw_radians);

#define EXPECT_BETWEEN_EXCLUSIVE(value, a, b) \
  {                                           \
    auto low = std::min(a, b);                \
    auto high = std::max(a, b);               \
    EXPECT_GT(value, low);                    \
    EXPECT_LT(value, high);                   \
  }

namespace {
// Expects angles to be normalized
double DeltaAngle(double a, double b) {
  double delta = std::abs(a - b);
  return std::min(delta, (2.0 * M_PI) - delta);
}
}  // namespace

// Expects angles to be normalized
#define EXPECT_ANGLE_BETWEEN_EXCLUSIVE(theta, a, b)  \
  EXPECT_LT(DeltaAngle(a, theta), DeltaAngle(a, b)); \
  EXPECT_LT(DeltaAngle(b, theta), DeltaAngle(a, b));

#define EXPECT_POSE_IN_RANGE(interpolated_pose, pose_start, pose_end)      \
  EXPECT_BETWEEN_EXCLUSIVE(interpolated_pose.x, pose_start.x, pose_end.x); \
  EXPECT_BETWEEN_EXCLUSIVE(interpolated_pose.y, pose_start.y, pose_end.y); \
  EXPECT_ANGLE_BETWEEN_EXCLUSIVE(interpolated_pose.yaw_radians,            \
                                 pose_start.yaw_radians,                   \
                                 pose_end.yaw_radians);

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
ceres::examples::Pose2d MakePose(double x, double y, double yaw_radians) {
  return ceres::examples::Pose2d{x, y, yaw_radians};
}

bool TargetIsInView(TargetMapper::TargetPose target_detection) {
  // And check if it is within the fov of the robot /
  // camera, assuming camera is pointing in the
  // positive x-direction of the robot
  double angle_to_target =
      atan2(target_detection.pose.y, target_detection.pose.x);

  // Simulated camera field of view, in radians
  constexpr double kCameraFov = M_PI_2;
  if (fabs(angle_to_target) <= kCameraFov / 2.0) {
    VLOG(2) << "Found target in view, based on T = " << target_detection.pose.x
            << ", " << target_detection.pose.y << " with angle "
            << angle_to_target;
    return true;
  } else {
    return false;
  }
}

aos::distributed_clock::time_point TimeInMs(size_t ms) {
  return aos::distributed_clock::time_point(std::chrono::milliseconds(ms));
}

}  // namespace

TEST(DataAdapterTest, MatchTargetDetections) {
  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses = {
      {TimeInMs(0), ceres::examples::Pose2d{1.0, 2.0, 0.0}},
      {TimeInMs(5), ceres::examples::Pose2d{1.0, 2.0, 0.0}},
      {TimeInMs(10), ceres::examples::Pose2d{3.0, 1.0, M_PI_2}},
      {TimeInMs(15), ceres::examples::Pose2d{5.0, -2.0, -M_PI}},
      {TimeInMs(20), ceres::examples::Pose2d{5.0, -2.0, -M_PI}},
      {TimeInMs(25), ceres::examples::Pose2d{10.0, -32.0, M_PI_2}},
      {TimeInMs(30), ceres::examples::Pose2d{-15.0, 12.0, 0.0}},
      {TimeInMs(35), ceres::examples::Pose2d{-15.0, 12.0, 0.0}}};
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {{TimeInMs(5),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        0},
       {TimeInMs(9),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        1},
       {TimeInMs(9),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        2},
       {TimeInMs(15),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        0},
       {TimeInMs(16),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        2},
       {TimeInMs(27),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, 0.0}),
        1}};
  auto [target_constraints, robot_delta_poses] =
      DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                         timestamped_target_detections);

  // Check that target constraints got inserted in the
  // correct spots
  EXPECT_EQ(target_constraints.size(),
            timestamped_target_detections.size() - 1);
  for (auto it = target_constraints.begin(); it < target_constraints.end();
       it++) {
    auto timestamped_it = timestamped_target_detections.begin() +
                          (it - target_constraints.begin());
    EXPECT_EQ(it->id_begin, timestamped_it->id);
    EXPECT_EQ(it->id_end, (timestamped_it + 1)->id);
  }

  // Check that poses were interpolated correctly.
  // Keep track of the computed robot pose by adding the delta poses
  auto computed_robot_pose = timestamped_robot_poses[1].pose;

  computed_robot_pose =
      PoseUtils::ComputeOffsetPose(computed_robot_pose, robot_delta_poses[0]);
  EXPECT_POSE_IN_RANGE(computed_robot_pose, timestamped_robot_poses[1].pose,
                       timestamped_robot_poses[2].pose);

  computed_robot_pose =
      PoseUtils::ComputeOffsetPose(computed_robot_pose, robot_delta_poses[1]);
  EXPECT_POSE_IN_RANGE(computed_robot_pose, timestamped_robot_poses[1].pose,
                       timestamped_robot_poses[2].pose);
  EXPECT_POSE_EQ(robot_delta_poses[1], MakePose(0.0, 0.0, 0.0));

  computed_robot_pose =
      PoseUtils::ComputeOffsetPose(computed_robot_pose, robot_delta_poses[2]);
  EXPECT_POSE_EQ(computed_robot_pose, timestamped_robot_poses[3].pose);

  computed_robot_pose =
      PoseUtils::ComputeOffsetPose(computed_robot_pose, robot_delta_poses[3]);
  EXPECT_POSE_EQ(computed_robot_pose, timestamped_robot_poses[4].pose);

  computed_robot_pose =
      PoseUtils::ComputeOffsetPose(computed_robot_pose, robot_delta_poses[4]);
  EXPECT_POSE_IN_RANGE(computed_robot_pose, timestamped_robot_poses[5].pose,
                       timestamped_robot_poses[6].pose);

  // Check the confidence matrices. Don't check the actual values
  // in case the constants change, just check the confidence of contraints
  // relative to each other, as constraints over longer time periods should have
  // lower confidence.
  const auto confidence_0ms =
      DataAdapter::ComputeConfidence(TimeInMs(0), TimeInMs(0));
  const auto confidence_1ms =
      DataAdapter::ComputeConfidence(TimeInMs(0), TimeInMs(1));
  const auto confidence_4ms =
      DataAdapter::ComputeConfidence(TimeInMs(0), TimeInMs(4));
  const auto confidence_6ms =
      DataAdapter::ComputeConfidence(TimeInMs(0), TimeInMs(6));
  const auto confidence_11ms =
      DataAdapter::ComputeConfidence(TimeInMs(0), TimeInMs(11));

  // Check relative magnitude of different confidences.
  // Confidences for 0-5ms, 5-10ms, and 10-15ms periods are equal
  // because they fit within one control loop iteration.
  EXPECT_EQ(confidence_0ms, confidence_1ms);
  EXPECT_EQ(confidence_1ms, confidence_4ms);
  EXPECT_CONFIDENCE_GT(confidence_4ms, confidence_6ms);
  EXPECT_CONFIDENCE_GT(confidence_6ms, confidence_11ms);

  // Check that confidences (information) of actual constraints are correct
  EXPECT_EQ(target_constraints[0].information, confidence_4ms);
  EXPECT_EQ(target_constraints[1].information, confidence_0ms);
  EXPECT_EQ(target_constraints[2].information, confidence_6ms);
  EXPECT_EQ(target_constraints[3].information, confidence_1ms);
  EXPECT_EQ(target_constraints[4].information, confidence_11ms);
}

TEST(DataAdapterTest, MatchTargetDetectionsWithoutRobotPosition) {
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {{TimeInMs(5),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -5.0, 0.0}),
        2},
       {TimeInMs(6),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{5.0, -4.0, M_PI}),
        0},
       {TimeInMs(10),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{3.0, -3.0, M_PI}),
        1},
       {TimeInMs(13),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{4.0, -7.0, M_PI_2}),
        2},
       {TimeInMs(14),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{4.0, -4.0, M_PI_2}),
        2}};

  constexpr auto kMaxDt = std::chrono::milliseconds(3);
  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_target_detections, kMaxDt);

  // The constraint between the detection at 6ms and the one at 10 ms is skipped
  // because dt > kMaxDt.
  // Also, the constraint between the last two detections is skipped because
  // they are the same target
  EXPECT_EQ(target_constraints.size(),
            timestamped_target_detections.size() - 3);

  // Between 5ms and 6ms detections
  EXPECT_DOUBLE_EQ(target_constraints[0].x, 0.0);
  EXPECT_DOUBLE_EQ(target_constraints[0].y, 1.0);
  EXPECT_DOUBLE_EQ(target_constraints[0].yaw_radians, -M_PI);
  EXPECT_EQ(target_constraints[0].id_begin, 2);
  EXPECT_EQ(target_constraints[0].id_end, 0);

  // Between 10ms and 13ms detections
  EXPECT_DOUBLE_EQ(target_constraints[1].x, -1.0);
  EXPECT_DOUBLE_EQ(target_constraints[1].y, 4.0);
  EXPECT_DOUBLE_EQ(target_constraints[1].yaw_radians, -M_PI_2);
  EXPECT_EQ(target_constraints[1].id_begin, 1);
  EXPECT_EQ(target_constraints[1].id_end, 2);
}

TEST(TargetMapperTest, TwoTargetsOneConstraint) {
  std::map<TargetMapper::TargetId, ceres::examples::Pose2d> target_poses;
  target_poses[0] = ceres::examples::Pose2d{5.0, 0.0, M_PI};
  target_poses[1] = ceres::examples::Pose2d{-5.0, 0.0, 0.0};

  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses = {
      {TimeInMs(5), ceres::examples::Pose2d{2.0, 0.0, 0.0}},
      {TimeInMs(10), ceres::examples::Pose2d{-1.0, 0.0, 0.0}},
      {TimeInMs(15), ceres::examples::Pose2d{-1.0, 0.0, 0.0}}};
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {{TimeInMs(5),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{3.0, 0.0, M_PI}),
        0},
       {TimeInMs(10),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{-4.0, 0.0, 0.0}),
        1}};
  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                         timestamped_target_detections)
          .first;

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], MakePose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], MakePose(-5.0, 0.0, 0.0));
}

TEST(TargetMapperTest, TwoTargetsTwoConstraints) {
  std::map<TargetMapper::TargetId, ceres::examples::Pose2d> target_poses;
  target_poses[0] = ceres::examples::Pose2d{5.0, 0.0, M_PI};
  target_poses[1] = ceres::examples::Pose2d{-5.0, 0.0, -M_PI_2};

  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses = {
      {TimeInMs(5), ceres::examples::Pose2d{-1.0, 0.0, 0.0}},
      {TimeInMs(10), ceres::examples::Pose2d{3.0, 0.0, 0.0}},
      {TimeInMs(15), ceres::examples::Pose2d{4.0, 0.0, 0.0}},
      {TimeInMs(20), ceres::examples::Pose2d{-1.0, 0.0, 0.0}}};
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {{TimeInMs(5),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{6.0, 0.0, M_PI}),
        0},
       {TimeInMs(10),
        PoseUtils::Pose2dToAffine3d(
            ceres::examples::Pose2d{-8.0, 0.0, -M_PI_2}),
        1},
       {TimeInMs(15),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{1.0, 0.0, M_PI}),
        0}};
  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                         timestamped_target_detections)
          .first;

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], MakePose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], MakePose(-5.0, 0.0, -M_PI_2));
}

TEST(TargetMapperTest, TwoTargetsOneNoisyConstraint) {
  std::map<TargetMapper::TargetId, ceres::examples::Pose2d> target_poses;
  target_poses[0] = ceres::examples::Pose2d{5.0, 0.0, M_PI};
  target_poses[1] = ceres::examples::Pose2d{-5.0, 0.0, 0.0};

  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses = {
      {TimeInMs(5), ceres::examples::Pose2d{1.99, 0.0, 0.0}},
      {TimeInMs(10), ceres::examples::Pose2d{-1.0, 0.0, 0.0}},
      {TimeInMs(15), ceres::examples::Pose2d{-1.01, -0.01, 0.004}}};
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections =
      {{TimeInMs(5),
        PoseUtils::Pose2dToAffine3d(
            ceres::examples::Pose2d{3.01, 0.001, M_PI - 0.001}),
        0},
       {TimeInMs(10),
        PoseUtils::Pose2dToAffine3d(ceres::examples::Pose2d{-4.01, 0.0, 0.0}),
        1}};

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                         timestamped_target_detections)
          .first;

  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  ASSERT_EQ(mapper.target_poses().size(), 2);
  EXPECT_POSE_NEAR(mapper.target_poses()[0], MakePose(5.0, 0.0, M_PI));
  EXPECT_POSE_NEAR(mapper.target_poses()[1], MakePose(-5.0, 0.0, 0.0));
}

TEST(TargetMapperTest, MultiTargetCircleMotion) {
  // Build set of target locations wrt global origin
  // For simplicity, do this on a grid of the field
  double field_half_length = 7.5;  // half length of the field
  double field_half_width = 5.0;   // half width of the field
  std::map<TargetMapper::TargetId, ceres::examples::Pose2d> target_poses;
  std::vector<TargetMapper::TargetPose> actual_target_poses;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      TargetMapper::TargetId target_id = i * 3 + j;
      TargetMapper::TargetPose target_pose{
          target_id, ceres::examples::Pose2d{field_half_length * (1 - i),
                                             field_half_width * (1 - j), 0.0}};
      actual_target_poses.emplace_back(target_pose);
      target_poses[target_id] = target_pose.pose;
      VLOG(2) << "VERTEX_SE2 " << target_id << " " << target_pose.pose.x << " "
              << target_pose.pose.y << " " << target_pose.pose.yaw_radians;
    }
  }

  // Now, create a bunch of robot poses and target
  // observations
  size_t dt = 1;

  std::vector<DataAdapter::TimestampedPose> timestamped_robot_poses;
  std::vector<DataAdapter::TimestampedDetection> timestamped_target_detections;

  constexpr size_t kTotalSteps = 100;
  for (size_t step_count = 0; step_count < kTotalSteps; step_count++) {
    size_t t = dt * step_count;
    // Circle clockwise around the center of the field
    double robot_theta = t;
    double robot_x = (field_half_length / 2.0) * cos(robot_theta);
    double robot_y = (-field_half_width / 2.0) * sin(robot_theta);

    ceres::examples::Pose2d robot_pose{robot_x, robot_y, robot_theta};
    for (TargetMapper::TargetPose target_pose : actual_target_poses) {
      TargetMapper::TargetPose target_detection = {
          .id = target_pose.id,
          .pose = PoseUtils::ComputeRelativePose(robot_pose, target_pose.pose)};
      if (TargetIsInView(target_detection)) {
        // Define random generator with Gaussian
        // distribution
        const double mean = 0.0;
        const double stddev = 1.0;
        // Can play with this to see how it impacts
        // randomness
        constexpr double kNoiseScale = 0.01;
        std::default_random_engine generator(aos::testing::RandomSeed());
        std::normal_distribution<double> dist(mean, stddev);

        target_detection.pose.x += dist(generator) * kNoiseScale;
        target_detection.pose.y += dist(generator) * kNoiseScale;
        robot_pose.x += dist(generator) * kNoiseScale;
        robot_pose.y += dist(generator) * kNoiseScale;

        auto time_point =
            aos::distributed_clock::time_point(std::chrono::milliseconds(t));
        timestamped_robot_poses.emplace_back(DataAdapter::TimestampedPose{
            .time = time_point, .pose = robot_pose});
        timestamped_target_detections.emplace_back(
            DataAdapter::TimestampedDetection{
                .time = time_point,
                .H_robot_target =
                    PoseUtils::Pose2dToAffine3d(target_detection.pose),
                .id = target_detection.id});
      }
    }
  }

  {
    // Add in a robot pose after all target poses
    auto final_robot_pose =
        timestamped_robot_poses[timestamped_robot_poses.size() - 1];
    timestamped_robot_poses.emplace_back(DataAdapter::TimestampedPose{
        .time = final_robot_pose.time + std::chrono::milliseconds(dt),
        .pose = final_robot_pose.pose});
  }

  auto target_constraints =
      DataAdapter::MatchTargetDetections(timestamped_robot_poses,
                                         timestamped_target_detections)
          .first;
  frc971::vision::TargetMapper mapper(target_poses, target_constraints);
  mapper.Solve(kFieldName);

  for (auto [target_pose_id, mapper_target_pose] : mapper.target_poses()) {
    TargetMapper::TargetPose actual_target_pose =
        TargetMapper::GetTargetPoseById(actual_target_poses, target_pose_id)
            .value();
    EXPECT_POSE_NEAR(mapper_target_pose, actual_target_pose.pose);
  }

  //
  // See what happens when we don't start with the
  // correct values
  //
  for (auto [target_id, target_pose] : target_poses) {
    // Skip first pose, since that needs to be correct
    // and is fixed in the solver
    if (target_id != 0) {
      ceres::examples::Pose2d bad_pose{0.0, 0.0, M_PI / 2.0};
      target_poses[target_id] = bad_pose;
    }
  }

  frc971::vision::TargetMapper mapper_bad_poses(target_poses,
                                                target_constraints);
  mapper_bad_poses.Solve(kFieldName);

  for (auto [target_pose_id, mapper_target_pose] :
       mapper_bad_poses.target_poses()) {
    TargetMapper::TargetPose actual_target_pose =
        TargetMapper::GetTargetPoseById(actual_target_poses, target_pose_id)
            .value();
    EXPECT_POSE_NEAR(mapper_target_pose, actual_target_pose.pose);
  }
}

}  // namespace frc971::vision
