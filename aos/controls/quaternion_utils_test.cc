#include "Eigen/Dense"

#include <random>

#include "aos/controls/quaternion_utils.h"
#include "aos/testing/random_seed.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace controls {
namespace testing {

// Tests that small perturbations around a couple quaternions averaged out
// return the original quaternion.
TEST(DownEstimatorTest, QuaternionMean) {
  Eigen::Matrix<double, 4, 7> vectors;
  vectors.col(0) << 0, 0, 0, 1;
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix<double, 4, 1> perturbation;
    perturbation.setZero();
    perturbation(i, 0) = 0.1;

    vectors.col(i * 2 + 1) = vectors.col(0) + perturbation;
    vectors.col(i * 2 + 2) = vectors.col(0) - perturbation;
  }

  for (int i = 0; i < 7; ++i) {
    vectors.col(i).normalize();
  }

  Eigen::Matrix<double, 4, 1> mean = QuaternionMean(vectors);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mean(i, 0), vectors(i, 0), 0.001) << ": Failed on index " << i;
  }
}

// Tests that ToRotationVectorFromQuaternion works for a 0 rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionAtZero) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::Zero()).norm(), 1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a real rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternion) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          Eigen::AngleAxis<double>(M_PI * 0.5, Eigen::Vector3d::UnitX()))
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a solution with negative
// coefficients.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionNegative) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          -Eigen::Quaternion<double>(
               Eigen::AngleAxis<double>(M_PI * 0.5, Eigen::Vector3d::UnitX()))
               .coeffs())
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a 0 rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVectorAtZero) {
  Eigen::Matrix<double, 4, 1> quaternion =
      ToQuaternionFromRotationVector(Eigen::Vector3d::Zero());

  EXPECT_NEAR(
      0.0,
      (quaternion - Eigen::Quaternion<double>(
                        Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
                        .coeffs())
          .norm(),
      1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a real rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVector) {
  Eigen::Matrix<double, 4, 1> quaternion =
      ToQuaternionFromRotationVector(Eigen::Vector3d::UnitX() * M_PI * 0.5);

  EXPECT_NEAR(0.0,
              (quaternion - Eigen::Quaternion<double>(
                                Eigen::AngleAxis<double>(
                                    M_PI * 0.5, Eigen::Vector3d::UnitX()))
                                .coeffs())

                  .norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector correctly clips a rotation vector
// that is too large in magnitude.
TEST(DownEstimatorTest, ToQuaternionFromLargeRotationVector) {
  const double kMaxAngle = 2.0;
  const Eigen::Vector3d rotation_vector =
      Eigen::Vector3d::UnitX() * kMaxAngle * 2.0;
  const Eigen::Matrix<double, 3, 1> clipped_vector =
      ToRotationVectorFromQuaternion(
          ToQuaternionFromRotationVector(rotation_vector, kMaxAngle));

  EXPECT_NEAR(0.0, (rotation_vector / 2.0 - clipped_vector).norm(), 1e-4);
}

// Tests that ToQuaternionFromRotationVector and ToRotationVectorFromQuaternion
// works for random rotations.
TEST(DownEstimatorTest, RandomQuaternions) {
  std::mt19937 generator(aos::testing::RandomSeed());
  std::uniform_real_distribution<double> random_scalar(-1.0, 1.0);

  for (int i = 0; i < 1000; ++i) {
    Eigen::Matrix<double, 3, 1> axis;
    axis << random_scalar(generator), random_scalar(generator),
        random_scalar(generator);
    EXPECT_GE(axis.norm(), 1e-6);
    axis.normalize();

    const double angle = random_scalar(generator) * M_PI;

    Eigen::Matrix<double, 4, 1> quaternion =
        ToQuaternionFromRotationVector(axis * angle);

    Eigen::Quaternion<double> answer(Eigen::AngleAxis<double>(angle, axis));

    EXPECT_NEAR(quaternion(3, 0), std::cos(angle / 2.0), 1e-8);
    EXPECT_NEAR(answer.w(), std::cos(angle / 2.0), 1e-8);

    EXPECT_NEAR(1.0, (answer.coeffs() * quaternion.transpose()).norm(), 1e-6);

    const Eigen::Matrix<double, 3, 1> recalculated_axis =
        ToRotationVectorFromQuaternion(quaternion);

    EXPECT_NEAR(std::abs(angle), recalculated_axis.norm(), 1e-8);

    EXPECT_NEAR(0.0, (axis * angle - recalculated_axis).norm(), 1e-8);
  }
}

}  // namespace testing
}  // namespace controls
}  // namespace aos
