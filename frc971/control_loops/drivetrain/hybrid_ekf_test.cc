#include "frc971/control_loops/drivetrain/hybrid_ekf.h"

#include <random>

#include "aos/testing/random_seed.h"
#include "aos/testing/test_shm.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/trajectory.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

typedef HybridEkf<>::StateIdx StateIdx;
typedef HybridEkf<>::InputIdx InputIdx;
typedef HybridEkf<>::State State;
typedef HybridEkf<>::StateSquare StateSquare;
typedef HybridEkf<>::Input Input;

class HybridEkfTest : public ::testing::Test {
 public:
  ::aos::testing::TestSharedMemory shm_;
  HybridEkfTest()
      : dt_config_(GetTestDrivetrainConfig()),
        ekf_(dt_config_),
        t0_(::std::chrono::seconds(0)),
        velocity_plant_coefs_(dt_config_.make_hybrid_drivetrain_velocity_loop()
                                  .plant()
                                  .coefficients()) {
    ekf_.ResetInitialState(t0_, State::Zero(), StateSquare::Identity());
  }

 protected:
  const State Update(const State &X, const Input &U, bool ignore_accel) {
    return RungeKuttaU(
        std::bind(&HybridEkfTest::DiffEq, this, std::placeholders::_1,
                  std::placeholders::_2, ignore_accel),
        X, U, aos::time::DurationInSeconds(dt_config_.dt));
  }
  void CheckDiffEq(const State &X, const Input &U, bool ignore_accel) {
    // Re-implement dynamics as a sanity check:
    const double diameter = 2.0 * dt_config_.robot_radius;
    const double theta = X(StateIdx::kTheta);
    const double stheta = std::sin(theta);
    const double ctheta = std::cos(theta);
    const double left_vel = X(StateIdx::kLeftVelocity);
    const double right_vel = X(StateIdx::kRightVelocity);
    const double lng_vel = (left_vel + right_vel) / 2.0;
    const double yaw_rate =
        (right_vel - left_vel) / 2.0 / dt_config_.robot_radius;
    const double lat_vel = X(StateIdx::kLateralVelocity);
    const State Xdot_ekf = DiffEq(X, U, ignore_accel);
    EXPECT_EQ(Xdot_ekf(StateIdx::kX), ctheta * lng_vel - stheta * lat_vel);
    EXPECT_EQ(Xdot_ekf(StateIdx::kY), stheta * lng_vel + ctheta * lat_vel);
    EXPECT_EQ(Xdot_ekf(StateIdx::kTheta), (right_vel - left_vel) / diameter);
    EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kLeftEncoder),
                    left_vel + X(StateIdx::kAngularError) -
                        X(StateIdx::kLongitudinalVelocityOffset));
    EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kRightEncoder),
                    right_vel - X(StateIdx::kAngularError) -
                        X(StateIdx::kLongitudinalVelocityOffset));

    const Eigen::Matrix<double, 2, 1> vel_x(X(StateIdx::kLeftVelocity),
                                            X(StateIdx::kRightVelocity));
    // Don't expect any contribution from the voltage terms, since we currently
    // disable them.
    const Eigen::Matrix<double, 2, 1> expected_accel =
        velocity_plant_coefs_.A_continuous * vel_x +
        velocity_plant_coefs_.B_continuous *
            (U.topRows(2) + X.block<2, 1>(StateIdx::kLeftVoltageError, 0));
    const double nominal_lng_accel =
        (expected_accel(0) + expected_accel(1)) / 2.0;
    if (ignore_accel) {
      EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kLeftVelocity), expected_accel(0));
      EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kRightVelocity), expected_accel(1));
      EXPECT_EQ(Xdot_ekf(StateIdx::kLateralVelocity), 0.0);
      EXPECT_EQ(
          Xdot_ekf(StateIdx::kLongitudinalVelocityOffset), 0.0);
    } else {
      const double lng_accel = U(InputIdx::kLongitudinalAccel) +
                               lat_vel * yaw_rate + ekf_.VelocityAccel(lng_vel);
      EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kLeftVelocity),
                      lng_accel + expected_accel(0) - nominal_lng_accel);
      EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kRightVelocity),
                      lng_accel + expected_accel(1) - nominal_lng_accel);

      EXPECT_FLOAT_EQ(Xdot_ekf(StateIdx::kLongitudinalVelocityOffset),
                      -X(StateIdx::kLongitudinalVelocityOffset) /
                          HybridEkf<>::kVelocityOffsetTimeConstant);
      const double centripetal_accel = lng_vel * yaw_rate;
      const double lat_accel = U(InputIdx::kLateralAccel) - centripetal_accel;
      EXPECT_EQ(Xdot_ekf(StateIdx::kLateralVelocity),
                lat_accel - X(StateIdx::kLateralVelocity) /
                                HybridEkf<>::kLateralVelocityTimeConstant);
    }

    // Dynamics don't expect error terms to change:
    EXPECT_EQ(0.0, Xdot_ekf(StateIdx::kLeftVoltageError));
    EXPECT_EQ(0.0, Xdot_ekf(StateIdx::kRightVoltageError));
    EXPECT_EQ(0.0, Xdot_ekf(StateIdx::kAngularError));
  }
  State DiffEq(const State &X, const Input &U, bool ignore_accel) {
    return ekf_.DiffEq(X, U, ignore_accel);
  }
  StateSquare AForState(const State &X, bool ignore_accel) {
    return ekf_.AForState(X, ignore_accel);
  }

  // Returns a random value sampled from a normal distribution with a standard
  // deviation of std and a mean of zero.
  double Normal(double std) { return normal_(gen_) * std; }

  DrivetrainConfig<double> dt_config_;
  HybridEkf<> ekf_;
  aos::monotonic_clock::time_point t0_;
  StateFeedbackHybridPlantCoefficients<2, 2, 2> velocity_plant_coefs_;

  std::mt19937 gen_{static_cast<uint32_t>(::aos::testing::RandomSeed())};
  std::normal_distribution<> normal_;
};


// Tests that if we provide a bunch of observations of the position
// with zero change in time, the state should approach the estimation.
struct DiffEqInputs {
  State X;
  Input U;
  bool ignore_accel;
};

DiffEqInputs MakeDiffEqInputs(State X, Input U, bool ignore_accel) {
  return {.X = X, .U = U, .ignore_accel = ignore_accel};
}

class HybridEkfDiffEqTest : public HybridEkfTest,
                            public ::testing::WithParamInterface<DiffEqInputs> {
};

// Tests that the dynamics from DiffEq behave as expected:
TEST_P(HybridEkfDiffEqTest, CheckDynamics) {
  CheckDiffEq(GetParam().X, GetParam().U, GetParam().ignore_accel);

  // Calculate whether A seems to be empirically correct.
  const StateSquare A = AForState(GetParam().X, GetParam().ignore_accel);
  for (int ii = 0; ii < HybridEkf<>::kNStates; ++ii) {
    SCOPED_TRACE(ii);
    State perturbation = State::Zero();
    constexpr double kEps = 1e-5;
    perturbation(ii) = kEps;
    const State Xdot_plus = DiffEq(GetParam().X + perturbation, GetParam().U,
                                   GetParam().ignore_accel);
    const State Xdot_minus = DiffEq(GetParam().X - perturbation, GetParam().U,
                                   GetParam().ignore_accel);
    const State numerical_dXdot_dX = (Xdot_plus - Xdot_minus) / (2.0 * kEps);
    const State A_based_dXdot_dX = A * perturbation / kEps;
    EXPECT_LT((A_based_dXdot_dX - numerical_dXdot_dX).norm(), 1e-8)
        << "A * X: " << A_based_dXdot_dX << " numerical: " << numerical_dXdot_dX
        << " difference: " << (A_based_dXdot_dX - numerical_dXdot_dX);
  }
}

INSTANTIATE_TEST_CASE_P(
    CheckMathTest, HybridEkfDiffEqTest,
    ::testing::Values(DiffEqInputs{State::Zero(), Input::Zero(), false},
                      DiffEqInputs{State::Zero(), Input::Zero(), true},
                      DiffEqInputs{State::Zero(), {-5.0, 5.0, 0.0, 0.0}, false},
                      DiffEqInputs{State::Zero(), {-5.0, 5.0, 0.0, 0.0}, true},
                      DiffEqInputs{State::Zero(), {12.0, 3.0, 0.0, 0.0}, false},
                      DiffEqInputs{State::Zero(), {12.0, 3.0, 0.0, 0.0}, true},
                      DiffEqInputs{(State() << 100.0, 200.0, M_PI, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   false},
                      DiffEqInputs{(State() << 100.0, 200.0, M_PI, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   true},
                      DiffEqInputs{(State() << 100.0, 200.0, 2.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   false},
                      DiffEqInputs{(State() << 100.0, 200.0, 2.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   true},
                      DiffEqInputs{(State() << 100.0, 200.0, 2.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.1, 0.2)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   false},
                      DiffEqInputs{(State() << 100.0, 200.0, -2.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {-3.0, -4.0, -5.0, -6.0},
                                   false},
                      DiffEqInputs{(State() << 100.0, 200.0, -2.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {-3.0, -4.0, -5.0, -6.0},
                                   true},
                      // And check that a theta outside of [-M_PI, M_PI] works.
                      DiffEqInputs{(State() << 100.0, 200.0, 200.0, 1.234, 0.5,
                                    1.2, 0.6, 3.0, -4.0, 0.3, 0.0, 0.0)
                                       .finished(),
                                   {3.0, 4.0, 5.0, 6.0},
                                   false}));

TEST_F(HybridEkfTest, ZeroTimeCorrect) {
  HybridEkf<>::Output Z(0.5, 0.5, 1);
  Eigen::Matrix<double, 3, 12> H;
  H.setIdentity();
  auto h = [H](const State &X, const Input &) { return H * X; };
  auto dhdx = [H](const State &) { return H; };
  Eigen::Matrix<double, 3, 3> R;
  R.setIdentity();
  R *= 1e-3;
  Input U = Input::Zero();
  EXPECT_EQ(0.0, ekf_.X_hat(StateIdx::kTheta));
  const double starting_p_norm = ekf_.P().norm();
  for (int ii = 0; ii < 100; ++ii) {
    ekf_.Correct(Z, &U, {}, h, dhdx, R, t0_);
  }
  EXPECT_NEAR(Z(0, 0), ekf_.X_hat(StateIdx::kX), 1e-3);
  EXPECT_NEAR(Z(1, 0), ekf_.X_hat(StateIdx::kY), 1e-3);
  EXPECT_NEAR(Z(2, 0), ekf_.X_hat(StateIdx::kTheta), 1e-3);
  const double ending_p_norm = ekf_.P().norm();
  // Due to corrections, noise should've decreased.
  EXPECT_LT(ending_p_norm, starting_p_norm * 0.95);
}

// Tests that prediction steps alone, with no corrections, results in sane
// results. In order to implement the "no corrections" part of that, we just set
// H to zero.
TEST_F(HybridEkfTest, PredictionsAreSane) {
  HybridEkf<>::Output Z(0, 0, 0);
  // Use true_X to track what we think the true robot state is.
  State true_X = ekf_.X_hat();
  Eigen::Matrix<double, 3, 12> H;
  H.setZero();
  auto h = [H](const State &X, const Input &) { return H * X; };
  auto dhdx = [H](const State &) { return H; };
  // Provide constant input voltage.
  Input U;
  U << 12.0, 10.0, 1.0, -0.1;
  // The exact value of the noise matrix is unimportant so long as it is
  // non-zero.
  Eigen::Matrix<double, 3, 3> R;
  R.setIdentity();
  EXPECT_EQ(0.0, ekf_.X_hat().norm());
  const double starting_p_norm = ekf_.P().norm();
  for (int ii = 0; ii < 100; ++ii) {
    ekf_.Correct(Z, &U, {}, h, dhdx, R, t0_ + dt_config_.dt * (ii + 1));
    true_X = Update(true_X, U, false);
    EXPECT_EQ(true_X, ekf_.X_hat());
  }
  // We don't care about precise results, just that they are generally sane:
  // robot should've travelled forwards and slightly to the right.
  EXPECT_NEAR(0.1, ekf_.X_hat(StateIdx::kX), 0.05);
  EXPECT_NEAR(-0.02, ekf_.X_hat(StateIdx::kY), 0.01);
  EXPECT_GT(0.01, ekf_.X_hat(StateIdx::kTheta));
  EXPECT_NEAR(0.3, ekf_.X_hat(StateIdx::kLeftEncoder), 0.1);
  EXPECT_NEAR(0.7, ekf_.X_hat(StateIdx::kLeftVelocity), 0.1);
  EXPECT_NEAR(0.3, ekf_.X_hat(StateIdx::kRightVelocity), 0.1);
  EXPECT_EQ(0.0, ekf_.X_hat(StateIdx::kLeftVoltageError));
  EXPECT_EQ(0.0, ekf_.X_hat(StateIdx::kRightVoltageError));
  EXPECT_EQ(0.0, ekf_.X_hat(StateIdx::kAngularError));
  // Without encoder updates, the longitudinal velocity offset should be zero.
  EXPECT_EQ(0.0, ekf_.X_hat(StateIdx::kLongitudinalVelocityOffset));
  EXPECT_NEAR(0.04, ekf_.X_hat(StateIdx::kLateralVelocity), 0.01);
  const double ending_p_norm = ekf_.P().norm();
  // Due to lack of corrections, noise should've increased.
  EXPECT_GT(ending_p_norm, starting_p_norm * 1.10);
}

// Parameterize HybridEkf off of the number of prediction-only updates to
// provide before doing measurements. This is so that we make sure to exercise
// the corner case when we apply measurements that appear at precisely the
// oldest time-step (during development, this corner case caused some issues).
class HybridEkfOldCorrectionsTest
    : public HybridEkfTest,
      public ::testing::WithParamInterface<size_t> {};

// Tests that creating an old measurement works, by basically running the
// previous two tests in reverse (running a series of predictions, and then
// inserting observation(s) at the start to change everything).
TEST_P(HybridEkfOldCorrectionsTest, CreateOldCorrection) {
  HybridEkf<>::Output Z;
  Z.setZero();
  Eigen::Matrix<double, 3, 12> H;
  H.setZero();
  auto h_zero = [H](const State &X, const Input &) { return H * X; };
  auto dhdx_zero = [H](const State &) { return H; };
  Input U;
  U << 12.0, 12.0, 0.0, 0.0;
  Eigen::Matrix<double, 3, 3> R;
  R = R.Identity();
  EXPECT_EQ(0.0, ekf_.X_hat().norm());
  // We fill up the buffer to be as full as demanded by the user.
  const size_t n_predictions = GetParam();
  for (size_t ii = 0; ii < n_predictions; ++ii) {
    ekf_.Correct(Z, &U, {}, h_zero, dhdx_zero, R,
                 t0_ + dt_config_.dt * (ii + 1));
  }

  // Store state and covariance after prediction steps.
  const State modeled_X_hat = ekf_.X_hat();
  const double modeled_p_norm = ekf_.P().norm();

  Z << 1, 1, M_PI / 2.0;
  H.setZero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = 1;
  auto h = [H](const State &X, const Input &) { return H * X; };
  auto dhdx = [H](const State &) { return H; };
  R.setZero();
  R.diagonal() << 1e-5, 1e-5, 1e-5;
  U.setZero();
  for (int ii = 0; ii < 20; ++ii) {
    ekf_.Correct(Z, &U, {}, h, dhdx, R, t0_);
  }
  const double corrected_p_norm = ekf_.P().norm();
  State expected_X_hat = modeled_X_hat;
  expected_X_hat(0, 0) = Z(0, 0);
  expected_X_hat(1, 0) = Z(1, 0) + modeled_X_hat(0, 0);
  expected_X_hat(2, 0) = Z(2, 0);
  EXPECT_LT((expected_X_hat.topRows<7>() - ekf_.X_hat().topRows<7>()).norm(),
            1e-3)
      << "X_hat: " << ekf_.X_hat() << " expected " << expected_X_hat;
  // The covariance after the predictions but before the corrections should
  // be higher than after the corrections are made.
  EXPECT_GT(modeled_p_norm, corrected_p_norm);
}

// Ensure that we check kSaveSamples - 1, for potential corner cases.
INSTANTIATE_TEST_CASE_P(OldCorrectionTest, HybridEkfOldCorrectionsTest,
                        ::testing::Values(0, 1, 10,
                                          HybridEkf<>::kSaveSamples - 1));

// Tests that creating a correction that is too old results in the correction
// being dropped and ignored.
TEST_F(HybridEkfTest, DiscardTooOldCorrection) {
  HybridEkf<>::Output Z;
  Z.setZero();
  Eigen::Matrix<double, 3, 12> H;
  H.setZero();
  auto h_zero = [H](const State &X, const Input &) { return H * X; };
  auto dhdx_zero = [H](const State &) { return H; };
  Input U;
  U << 12.0, 12.0, 0.0, 0.0;
  Eigen::Matrix<double, 3, 3> R;
  R.setIdentity();

  EXPECT_EQ(0.0, ekf_.X_hat().norm());
  for (int ii = 0; ii < HybridEkf<>::kSaveSamples; ++ii) {
    ekf_.Correct(Z, &U, {}, h_zero, dhdx_zero, R,
                 t0_ + dt_config_.dt * (ii + 1));
  }
  const State modeled_X_hat = ekf_.X_hat();
  const HybridEkf<>::StateSquare modeled_P = ekf_.P();

  Z << 1, 1, M_PI / 2.0;
  H.setZero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = 1;
  auto h = [H](const State &X, const Input &) { return H * X; };
  auto dhdx = [H](const State &) { return H; };
  R.setIdentity();
  R *= 1e-5;
  U.setZero();
  ekf_.Correct(Z, &U, {}, h, dhdx, R, t0_);
  EXPECT_EQ(ekf_.X_hat(), modeled_X_hat)
      << "Expected too-old correction to have no effect; X_hat: "
      << ekf_.X_hat() << " expected " << modeled_X_hat;
  EXPECT_EQ(ekf_.P(), modeled_P)
      << "Expected too-old correction to have no effect; P: " << ekf_.P()
      << " expected " << modeled_P;
}

// Tests The UpdateEncodersAndGyro function works when perfect measurements are
// provided:
TEST_F(HybridEkfTest, PerfectEncoderUpdate) {
  State true_X = ekf_.X_hat();
  Input U;
  U << -1.0, 5.0, 0.0, 0.0;
  for (int ii = 0; ii < 100; ++ii) {
    true_X = Update(true_X, U, false);
    ekf_.RawUpdateEncodersAndGyro(true_X(StateIdx::kLeftEncoder, 0),
                                  true_X(StateIdx::kRightEncoder, 0),
                                  (true_X(StateIdx::kRightVelocity, 0) -
                                   true_X(StateIdx::kLeftVelocity, 0)) /
                                      dt_config_.robot_radius / 2.0,
                                  U, t0_ + (ii + 1) * dt_config_.dt);
    EXPECT_NEAR((true_X - ekf_.X_hat()).squaredNorm(), 0.0, 1e-25)
        << "Expected only floating point precision errors in update step. "
           "Estimated X_hat:\n"
        << ekf_.X_hat() << "\ntrue X:\n"
        << true_X;
  }
}

// Tests that if we have an unusually large gap between measurement updates that
// nothing crazy happens (a previous implementation had a bug where large
// time-steps would mess up the prediction step due to how we approximated
// things).
TEST_F(HybridEkfTest, ExtraLongUpdateTime) {
  Input U;
  U << 0.0, 0.0, 0.1, 0.1;
  ekf_.RawUpdateEncodersAndGyro(0.0, 0.0, 0.0, U, t0_ + dt_config_.dt);
  ekf_.RawUpdateEncodersAndGyro(0.0, 0.0, 0.0, U,
                                t0_ + std::chrono::seconds(1000));
  EXPECT_LT(ekf_.X_hat().norm(), 10.0) << ekf_.X_hat();
}

// Tests encoder/gyro updates when we have some errors in our estimate.
TEST_F(HybridEkfTest, PerfectEncoderUpdateConverges) {
  // In order to simulate modelling errors, we add an angular_error and start
  // the encoder values slightly off.
  State true_X = ekf_.X_hat();
  true_X(StateIdx::kAngularError, 0) = 1.0;
  true_X(StateIdx::kLeftEncoder, 0) += 2.0;
  true_X(StateIdx::kRightEncoder, 0) -= 2.0;
  // After enough time, everything should converge to near-perfect (if there
  // were any errors in the original absolute state (x/y/theta) state, then we
  // can't correct for those).
  // Note: Because we don't have any absolute measurements used for corrections,
  // we will get slightly off on the absolute x/y/theta, but the errors are so
  // small they are negligible.
  Input U;
  U << 10.0, 5.0, 0.0, 0.0;
  for (int ii = 0; ii < 100; ++ii) {
    true_X = Update(true_X, U, false);
    ekf_.RawUpdateEncodersAndGyro(true_X(StateIdx::kLeftEncoder, 0),
                                  true_X(StateIdx::kRightEncoder, 0),
                                  (true_X(StateIdx::kRightVelocity, 0) -
                                   true_X(StateIdx::kLeftVelocity, 0)) /
                                      dt_config_.robot_radius / 2.0,
                                  U, t0_ + (ii + 1) * dt_config_.dt);
  }
  EXPECT_NEAR((true_X - ekf_.X_hat()).norm(), 0.0, 3e-5)
      << "Expected non-x/y estimates to converge to correct. "
         "Estimated X_hat:\n"
      << ekf_.X_hat() << "\ntrue X:\n"
      << true_X;
}

// Tests encoder/gyro updates in a realistic-ish scenario with noise:
TEST_F(HybridEkfTest, RealisticEncoderUpdateConverges) {
  // In order to simulate modelling errors, we add an angular_error and start
  // the encoder values slightly off.
  State true_X = ekf_.X_hat();
  true_X(StateIdx::kAngularError, 0) = 1.0;
  true_X(StateIdx::kLeftEncoder, 0) += 2.0;
  true_X(StateIdx::kRightEncoder, 0) -= 2.0;
  Input U;
  U << 10.0, 5.0, 0.0, 0.0;
  for (int ii = 0; ii < 100; ++ii) {
    true_X = Update(true_X, U, false);
    ekf_.RawUpdateEncodersAndGyro(
        true_X(StateIdx::kLeftEncoder, 0) + Normal(1e-3),
        true_X(StateIdx::kRightEncoder, 0) + Normal(1e-3),
        (true_X(StateIdx::kRightVelocity, 0) -
         true_X(StateIdx::kLeftVelocity, 0)) /
                dt_config_.robot_radius / 2.0 +
            Normal(1e-4),
        U, t0_ + (ii + 1) * dt_config_.dt);
  }
  EXPECT_NEAR(
      (true_X.bottomRows<9>() - ekf_.X_hat().bottomRows<9>()).squaredNorm(),
      0.0, 2e-3)
      << "Expected non-x/y estimates to converge to correct. "
         "Estimated X_hat:\n"
      << ekf_.X_hat() << "\ntrue X:\n"
      << true_X;
}

class HybridEkfDeathTest : public HybridEkfTest {};

TEST_F(HybridEkfDeathTest, DieIfUninitialized) {
  HybridEkf<> ekf(dt_config_);
  // Expect death if we fail to initialize before starting to provide updates.
  EXPECT_DEATH(
      ekf.RawUpdateEncodersAndGyro(0.0, 1.0, 2.0, {3.0, 3.0, 0.0, 0.0}, t0_),
      "observations_.empty()");
}

TEST_F(HybridEkfDeathTest, DieOnNoU) {
  // Expect death if the user does not provide U when creating a fresh
  // measurement.
  EXPECT_DEATH(ekf_.Correct({1, 2, 3}, nullptr, {}, {}, {}, {},
                            t0_ + std::chrono::seconds(1)),
               "U != nullptr");
}

// Because the user can choose to provide only one of make_h or (h, dhdx), check
// that we die when an improper combination is provided.
TEST_F(HybridEkfDeathTest, DieOnNoH) {
  // Check that we die when no h-related functions are provided:
  Input U;
  U << 1.0, 2.0, 0.0, 0.0;
  EXPECT_DEATH(ekf_.Correct({1, 2, 3}, &U, {}, {}, {}, {},
                            t0_ + std::chrono::seconds(1)),
               "make_h");
  // Check that we die when only one of h and dhdx are provided:
  EXPECT_DEATH(ekf_.Correct({1, 2, 3}, &U, {}, {},
                            [](const State &) {
                              return Eigen::Matrix<double, 3, 12>::Zero();
                            },
                            {}, t0_ + std::chrono::seconds(1)),
               "make_h");
  EXPECT_DEATH(ekf_.Correct({1, 2, 3}, &U, {},
                            [](const State &, const Input &) {
                              return Eigen::Matrix<double, 3, 1>::Zero();
                            },
                            {}, {}, t0_ + std::chrono::seconds(1)),
               "make_h");
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
