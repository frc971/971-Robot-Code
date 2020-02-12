#include "frc971/control_loops/runge_kutta.h"

#include <Eigen/Geometry>
#include <random>

#include "aos/testing/random_seed.h"
#include "frc971/control_loops/drivetrain/improved_down_estimator.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

namespace {
// Check if two quaternions are logically equal, to within some reasonable
// tolerance. This is needed because a single rotation can be represented by two
// quaternions.
bool QuaternionEqual(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b,
                     double tolerance) {
  // If a == b, then a.inverse() * b will be the identity. The identity
  // quaternion is the only time where the vector portion of the quaternion is
  // zero.
  return (a.inverse() * b).vec().norm() <= tolerance;
}
}  // namespace

// Do a known transformation to see if quaternion integration is working
// correctly.
TEST(DownEstimatorTest, QuaternionIntegral) {
  Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
  Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
  Eigen::Vector3d uz = Eigen::Vector3d::UnitZ();

  Eigen::Quaternion<double> q(
      Eigen::AngleAxis<double>(0.5 * M_PI, Eigen::Vector3d::UnitY()));

  Eigen::Quaternion<double> q0(
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()));

  auto qux = q * ux;

  VLOG(1) << "Q is w: " << q.w() << " vec: " << q.vec();
  VLOG(1) << "ux is " << ux;
  VLOG(1) << "qux is " << qux;

  // Start by rotating around the X body vector for pi/2
  Eigen::Quaternion<double> integral1(
      RungeKutta(std::bind(&drivetrain::DrivetrainUkf::QuaternionDerivative, ux,
                           std::placeholders::_1),
                 q0.coeffs(), 0.5 * M_PI));

  VLOG(1) << "integral1 * uz => " << integral1 * uz;

  // Then rotate around the Y body vector for pi/2
  Eigen::Quaternion<double> integral2(
      RungeKutta(std::bind(&drivetrain::DrivetrainUkf::QuaternionDerivative, uy,
                           std::placeholders::_1),
                 integral1.normalized().coeffs(), 0.5 * M_PI));

  VLOG(1) << "integral2 * uz => " << integral2 * uz;

  // Then rotate around the X body vector for -pi/2
  Eigen::Quaternion<double> integral3(
      RungeKutta(std::bind(&drivetrain::DrivetrainUkf::QuaternionDerivative,
                           -ux, std::placeholders::_1),
                 integral2.normalized().coeffs(), 0.5 * M_PI));

  integral1.normalize();
  integral2.normalize();
  integral3.normalize();

  VLOG(1) << "Integral is w: " << integral1.w() << " vec: " << integral1.vec()
          << " norm " << integral1.norm();

  VLOG(1) << "Integral is w: " << integral3.w() << " vec: " << integral3.vec()
          << " norm " << integral3.norm();

  VLOG(1) << "ux => " << integral3 * ux;
  EXPECT_NEAR(0.0, (ux - integral1 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral1 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (-uy - integral1 * uz).norm(), 5e-2);

  EXPECT_NEAR(0.0, (uy - integral2 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral2 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (ux - integral2 * uz).norm(), 5e-2);

  EXPECT_NEAR(0.0, (uy - integral3 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (-ux - integral3 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral3 * uz).norm(), 5e-2);
}

TEST(DownEstimatorTest, UkfConstantRotation) {
  drivetrain::DrivetrainUkf dtukf;
  const Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
  EXPECT_EQ(0.0,
            (Eigen::Vector3d(0.0, 0.0, 1.0) - dtukf.H(dtukf.X_hat().coeffs()))
                .norm());
  Eigen::Matrix<double, 3, 1> measurement;
  measurement.setZero();
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.Predict(ux * M_PI_2, measurement, std::chrono::milliseconds(5));
  }
  const Eigen::Quaterniond expected(Eigen::AngleAxis<double>(M_PI_2, ux));
  EXPECT_TRUE(QuaternionEqual(expected, dtukf.X_hat(), 0.01))
      << "Expected: " << expected.coeffs()
      << " Got: " << dtukf.X_hat().coeffs();
  EXPECT_NEAR(0.0,
            (Eigen::Vector3d(0.0, 1.0, 0.0) - dtukf.H(dtukf.X_hat().coeffs()))
                .norm(), 1e-10);
}

// Tests that the euler angles in the status message are correct.
TEST(DownEstimatorTest, UkfEulerStatus) {
  drivetrain::DrivetrainUkf dtukf;
  const Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d uz = Eigen::Vector3d::UnitZ();
  // First, rotate 3 radians in the yaw axis, then 0.5 radians in the pitch
  // axis, and then 0.1 radians about the roll axis.
  // The down estimator should ignore any of the pitch movement.
  constexpr double kYaw = 3.0;
  constexpr double kPitch = 0.5;
  constexpr double kRoll = 0.1;
  Eigen::Matrix<double, 3, 1> measurement;
  measurement.setZero();
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();
  const std::chrono::milliseconds dt(5);
  // Run a bunch of one-second rotations at the appropriate rate to cause the
  // total pitch/roll/yaw to be kPitch/kRoll/kYaw.
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.UpdateIntegratedPositions(now);
    now += dt;
    dtukf.Predict(uz * kYaw, measurement, dt);
  }
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.UpdateIntegratedPositions(now);
    now += dt;
    dtukf.Predict(uy * kPitch, measurement, dt);
  }
  EXPECT_FLOAT_EQ(kYaw, dtukf.yaw());
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.UpdateIntegratedPositions(now);
    now += dt;
    dtukf.Predict(ux * kRoll, measurement, dt);
  }
  EXPECT_FLOAT_EQ(kYaw, dtukf.yaw());
  const Eigen::Quaterniond expected(Eigen::AngleAxis<double>(kPitch, uy) *
                                    Eigen::AngleAxis<double>(kRoll, ux));
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(dtukf.PopulateStatus(&fbb, now));

  aos::FlatbufferDetachedBuffer<drivetrain::DownEstimatorState> state(
      fbb.Release());
  EXPECT_EQ(kPitch, state.message().longitudinal_pitch());
  // The longitudinal pitch is not actually the same number as the roll, so we
  // don't check it here.

  EXPECT_TRUE(QuaternionEqual(expected, dtukf.X_hat(), 0.0001))
      << "Expected: " << expected.coeffs()
      << " Got: " << dtukf.X_hat().coeffs();
}


// Tests that if the gyro indicates no movement but that the accelerometer shows
// that we are slightly rotated, that we eventually adjust our estimate to be
// correct.
TEST(DownEstimatorTest, UkfAccelCorrectsBias) {
  drivetrain::DrivetrainUkf dtukf;
  const Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
  Eigen::Matrix<double, 3, 1> measurement;
  // Supply the accelerometer with a slightly off reading to ensure that we
  // don't require exactly 1g to work.
  measurement << 0.01, 0.99, 0.0;
  EXPECT_TRUE(
      QuaternionEqual(Eigen::Quaterniond::Identity(), dtukf.X_hat(), 0.0))
      << "X_hat: " << dtukf.X_hat().coeffs();
  EXPECT_EQ(0.0,
            (Eigen::Vector3d(0.0, 0.0, 1.0) - dtukf.H(dtukf.X_hat().coeffs()))
                .norm());
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.Predict({0.0, 0.0, 0.0}, measurement, std::chrono::milliseconds(5));
  }
  const Eigen::Quaterniond expected(Eigen::AngleAxis<double>(M_PI_2, ux));
  EXPECT_TRUE(QuaternionEqual(expected, dtukf.X_hat(), 0.01))
      << "Expected: " << expected.coeffs()
      << " Got: " << dtukf.X_hat().coeffs();
}

// Tests that if the accelerometer is reading values with a magnitude that isn't ~1g,
// that we are slightly rotated, that we eventually adjust our estimate to be
// correct.
TEST(DownEstimatorTest, UkfIgnoreBadAccel) {
  drivetrain::DrivetrainUkf dtukf;
  const Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
  Eigen::Matrix<double, 3, 1> measurement;
  // Set up a scenario where, if we naively took the accelerometer readings, we
  // would think that we were rotated. But the gyro readings indicate that we
  // are only rotating about the Y (pitch) axis.
  measurement << 0.3, 1.0, 0.0;
  for (int ii = 0; ii < 200; ++ii) {
    dtukf.Predict({0.0, M_PI_2, 0.0}, measurement,
                  std::chrono::milliseconds(5));
  }
  const Eigen::Quaterniond expected(Eigen::AngleAxis<double>(M_PI_2, uy));
  EXPECT_TRUE(QuaternionEqual(expected, dtukf.X_hat(), 1e-1))
      << "Expected: " << expected.coeffs()
      << " Got: " << dtukf.X_hat().coeffs();
  EXPECT_NEAR(
      0.0,
      (Eigen::Vector3d(-1.0, 0.0, 0.0) - dtukf.H(dtukf.X_hat().coeffs()))
          .norm(),
      1e-10)
      << dtukf.H(dtukf.X_hat().coeffs());
}

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

  Eigen::Matrix<double, 4, 1> mean = drivetrain::QuaternionMean(vectors);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mean(i, 0), vectors(i, 0), 0.001) << ": Failed on index " << i;
  }
}

// Tests that computing sigma points, and then computing the mean and covariance
// returns the original answer.
TEST(DownEstimatorTest, SigmaPoints) {
  const Eigen::Quaternion<double> mean(
      Eigen::AngleAxis<double>(M_PI / 2.0, Eigen::Vector3d::UnitX()));

  Eigen::Matrix<double, 3, 3> covariance;
  covariance << 0.4, -0.1, 0.2, -0.1, 0.6, 0.0, 0.2, 0.0, 0.5;
  covariance *= 0.1;

  const Eigen::Matrix<double, 4, 3 * 2 + 1> vectors =
      drivetrain::GenerateSigmaPoints(mean, covariance);

  const Eigen::Matrix<double, 4, 1> calculated_mean =
      drivetrain::QuaternionMean(vectors);

  VLOG(1) << "actual mean: " << mean.coeffs();
  VLOG(1) << "calculated mean: " << calculated_mean;

  Eigen::Matrix<double, 3, 3 * 2 + 1> Wprime;
  Eigen::Matrix<double, 3, 3> calculated_covariance =
      drivetrain::ComputeQuaternionCovariance(
          Eigen::Quaternion<double>(calculated_mean), vectors, &Wprime);

  EXPECT_NEAR(1.0,
              (mean.conjugate().coeffs() * calculated_mean.transpose()).norm(),
              1e-4);

  EXPECT_NEAR(0.0, (calculated_covariance - covariance).norm(), 1e-8);
}

// Tests that computing sigma points with a large covariance that will precisely
// wrap, that we do clip the perturbations.
TEST(DownEstimatorTest, ClippedSigmaPoints) {
  const Eigen::Quaternion<double> mean(
      Eigen::AngleAxis<double>(M_PI / 2.0, Eigen::Vector3d::UnitX()));

  Eigen::Matrix<double, 3, 3> covariance;
  covariance << 0.4, -0.1, 0.2, -0.1, 0.6, 0.0, 0.2, 0.0, 0.5;
  covariance *= 100.0;

  const Eigen::Matrix<double, 4, 3 * 2 + 1> vectors =
      drivetrain::GenerateSigmaPoints(mean, covariance);

  const Eigen::Matrix<double, 4, 1> calculated_mean =
      drivetrain::QuaternionMean(vectors);

  Eigen::Matrix<double, 3, 3 * 2 + 1> Wprime;
  Eigen::Matrix<double, 3, 3> calculated_covariance =
      drivetrain::ComputeQuaternionCovariance(
          Eigen::Quaternion<double>(calculated_mean), vectors, &Wprime);

  EXPECT_NEAR(1.0,
              (mean.conjugate().coeffs() * calculated_mean.transpose()).norm(),
              1e-4);

  const double calculated_covariance_norm = calculated_covariance.norm();
  const double covariance_norm = covariance.norm();
  EXPECT_LT(calculated_covariance_norm, covariance_norm / 2.0)
      << "Calculated covariance should be much smaller than the original "
         "covariance.";
}

// Tests that ToRotationVectorFromQuaternion works for a 0 rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionAtZero) {
  Eigen::Matrix<double, 3, 1> vector =
      drivetrain::ToRotationVectorFromQuaternion(
          Eigen::Quaternion<double>(
              Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
              .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::Zero()).norm(), 1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a real rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternion) {
  Eigen::Matrix<double, 3, 1> vector =
      drivetrain::ToRotationVectorFromQuaternion(
          Eigen::Quaternion<double>(
              Eigen::AngleAxis<double>(M_PI * 0.5, Eigen::Vector3d::UnitX()))
              .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a solution with negative
// coefficients.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionNegative) {
  Eigen::Matrix<double, 3, 1> vector =
      drivetrain::ToRotationVectorFromQuaternion(
          Eigen::Quaternion<double>(
              -Eigen::Quaternion<double>(
                   Eigen::AngleAxis<double>(M_PI * 0.5,
                                              Eigen::Vector3d::UnitX()))
                   .coeffs())
              .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a 0 rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVectorAtZero) {
  Eigen::Matrix<double, 4, 1> quaternion =
      drivetrain::ToQuaternionFromRotationVector(Eigen::Vector3d::Zero());

  EXPECT_NEAR(0.0, (quaternion -
                    Eigen::Quaternion<double>(
                        Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
                        .coeffs()).norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a real rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVector) {
  Eigen::Matrix<double, 4, 1> quaternion =
      drivetrain::ToQuaternionFromRotationVector(Eigen::Vector3d::UnitX() *
                                                 M_PI * 0.5);

  EXPECT_NEAR(0.0, (quaternion -
                    Eigen::Quaternion<double>(
                        Eigen::AngleAxis<double>(
                            M_PI * 0.5, Eigen::Vector3d::UnitX())).coeffs())

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
      drivetrain::ToRotationVectorFromQuaternion(
          drivetrain::ToQuaternionFromRotationVector(rotation_vector,
                                                     kMaxAngle));

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
        drivetrain::ToQuaternionFromRotationVector(axis * angle);

    Eigen::Quaternion<double> answer(Eigen::AngleAxis<double>(angle, axis));

    EXPECT_NEAR(quaternion(3, 0), std::cos(angle / 2.0), 1e-8);
    EXPECT_NEAR(answer.w(), std::cos(angle / 2.0), 1e-8);

    EXPECT_NEAR(1.0, (answer.coeffs() * quaternion.transpose()).norm(), 1e-6);

    const Eigen::Matrix<double, 3, 1> recalculated_axis =
        drivetrain::ToRotationVectorFromQuaternion(quaternion);

    EXPECT_NEAR(std::abs(angle), recalculated_axis.norm(), 1e-8);

    EXPECT_NEAR(0.0, (axis * angle - recalculated_axis).norm(), 1e-8);
  }
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
