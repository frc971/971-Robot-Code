#include "frc971/vision/vision_util_lib.h"

#include "absl/strings/str_format.h"
#include "gtest/gtest.h"

#include "aos/util/math.h"

namespace frc971::vision {
// For now, just testing extracting camera number from channel name
TEST(VisionUtilsTest, CameraNumberFromChannel) {
  ASSERT_EQ(CameraNumberFromChannel("/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/camera"), std::nullopt);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/orin1"), std::nullopt);
}

namespace {
constexpr double kToleranceRadians = 0.05;
// Conversions between euler angles and quaternion result in slightly-off
// doubles
constexpr double kOrientationEqTolerance = 1e-10;
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

}  // namespace frc971::vision
