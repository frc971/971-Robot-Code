#include "frc971/estimation/ekf.h"

#include "gtest/gtest.h"

namespace frc971::control_loops::testing {
// Create some simple dynamics where the state is
// [position
//  velocity]
// and the input is [acceleration]
class PositionVelocityDynamics
    : public control_loops::swerve::DynamicsInterface<double, 2, 1> {
 public:
  using DynamicsInterface<double, 2, 1>::State;
  using DynamicsInterface<double, 2, 1>::Input;
  using DynamicsInterface<double, 2, 1>::StateSquare;
  State operator()(const State &X, const Input &U) const final {
    return Eigen::Vector2d{{X(1)}, {U(0)}};
  }
};

class EkfTest : public ::testing::Test {
 protected:
  using State = PositionVelocityDynamics::State;
  using Input = PositionVelocityDynamics::Input;
  using StateSquare = PositionVelocityDynamics::StateSquare;
  EkfTest()
      : ekf_({Eigen::Matrix2d{{1.0, 0.0}, {0.0, 1.0}},
              std::make_unique<PositionVelocityDynamics>()}) {}

  void DoMeaninglessMeasurementCorrection(aos::monotonic_clock::time_point now,
                                          const Input &input) {
    ekf_.Correct<1>(
        now, Eigen::Matrix<double, 1, 1>{{0.0}},
        Eigen::Matrix<double, 1, 1>{{std::numeric_limits<double>::infinity()}},
        input,
        [](const Eigen::Vector2d &) {
          return Eigen::Matrix<double, 1, 1>::Zero();
        },
        Eigen::Matrix<double, 1, 2>::Zero());
  }

  typedef Ekf<double, 2, 1>::CorrectionDebug<1> CorrectionDebug;

  CorrectionDebug DoPositionCorrection(aos::monotonic_clock::time_point now,
                                       const Input &input, double position,
                                       double noise = 1.0) {
    const Eigen::Matrix<double, 1, 2> H{{1.0, 0.0}};
    return ekf_.Correct<1>(
        now, Eigen::Matrix<double, 1, 1>{{position}},
        Eigen::Matrix<double, 1, 1>{{noise}}, input,
        [H](const Eigen::Vector2d &X) { return H * X; }, H);
  }

  Ekf<double, 2, 1> ekf_;
};

// Test that when we provide basic inputs the EKF exactly tracks the provided
// model.
TEST_F(EkfTest, FollowsNominalModel) {
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();
  Eigen::Vector2d true_state = Eigen::Vector2d::Zero();
  double input = 0.0;
  ekf_.Initialize(now, true_state, Eigen::Matrix2d::Identity());
  const std::chrono::seconds dt{1};
  now += dt;
  // State should not have changed since velocity started at zero and we
  // provided no input.
  DoMeaninglessMeasurementCorrection(now, Input{input});
  EXPECT_EQ(true_state, ekf_.X_hat());

  input = 1.0;
  now += dt;
  true_state(0) += 0.5 * input * std::pow(aos::time::DurationInSeconds(dt), 2);
  true_state(1) += input * aos::time::DurationInSeconds(dt);

  DoMeaninglessMeasurementCorrection(now, Input{input});
  EXPECT_EQ(true_state, ekf_.X_hat());
}

// Test that when we provide a correction that the state does not change if it
// doesn't need to.
TEST_F(EkfTest, CorrectionsHaveZeroImpactWhenModelIsPerfect) {
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();
  Eigen::Vector2d true_state = Eigen::Vector2d::Zero();
  double input = 0.0;
  ekf_.Initialize(now, true_state, Eigen::Matrix2d::Identity());
  CorrectionDebug debug = DoPositionCorrection(now, Input{input}, 0.0);
  EXPECT_EQ(0.0, debug.measurement(0));
  EXPECT_EQ(0.0, debug.expected(0));
  EXPECT_EQ(true_state, ekf_.X_hat());
}

// Test that when we provide a correction that the state changes, and that the
// correction noise impacts the magnitude of said updates.
TEST_F(EkfTest, PositionCorrections) {
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();
  Eigen::Vector2d true_state{{1.0, 0.0}};
  double input = 0.0;
  ekf_.Initialize(now, Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());
  const CorrectionDebug debug_noise_1 =
      DoPositionCorrection(now, Input{input}, true_state(0), 1.0);

  const Eigen::Vector2d state_correct_1 = ekf_.X_hat();

  ekf_.Initialize(now, Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());
  const CorrectionDebug debug_noise_2 =
      DoPositionCorrection(now, Input{input}, true_state(0), 2.0);
  const Eigen::Vector2d state_correct_2 = ekf_.X_hat();
  EXPECT_EQ(1.0, debug_noise_1.measurement(0));
  EXPECT_EQ(0.0, debug_noise_1.expected(0));
  EXPECT_EQ(1.0, debug_noise_2.measurement(0));
  EXPECT_EQ(0.0, debug_noise_2.expected(0));

  // Confirm that the positions were corrected by non-trivial values and that
  // state_correct_2 was lower in magnitude than state_correct_1.
  EXPECT_LT(0.1, state_correct_2(0));
  EXPECT_LT(state_correct_2(0) + 0.1, state_correct_1(0));
  EXPECT_LT(state_correct_1(0) + 0.1, true_state(0));

  // Because the estimate covariance is a diagonal matrix currently, the
  // velocity estimate does not actually update with a position correction.
  EXPECT_EQ(0.0, state_correct_1(1));
  EXPECT_EQ(0.0, state_correct_1(1));
}

// Test that position updates performed after the filter has stablized result in
// both position and velocity states getting updated appropriately.
TEST_F(EkfTest, PositionCorrectionsInEquilibrium) {
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();
  double input = 0.0;
  ekf_.Initialize(now, Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());
  Eigen::Matrix2d last_P = ekf_.P();
  // Iterate the filter a bunch to get it into a reasonable equilibrium.
  for (size_t ii = 0; ii < 100; ++ii) {
    now += std::chrono::seconds(1);
    last_P = ekf_.P();
    DoPositionCorrection(now, Input{input}, 0.0);
  }
  EXPECT_LT((last_P - ekf_.P()).norm(), 1e-5)
      << ": estimate covariance should have stablized.";
  // Do a position correction; this should result in non-trivial correction to
  // the velocity at first as well because we have off-diagonal covariance
  // terms.
  DoPositionCorrection(now, Input{input}, 1.0);
  EXPECT_LT(0.1, ekf_.X_hat()(0));
  EXPECT_LT(0.1, ekf_.X_hat()(1));
  // Do a bunch more corrections; we should stabliize back down to a state of
  // (1.0, 0.0).
  for (size_t ii = 0; ii < 100; ++ii) {
    now += std::chrono::seconds(1);
    DoPositionCorrection(now, Input{input}, 1.0);
  }
  EXPECT_NEAR(1.0, ekf_.X_hat()(0), 1e-10);
  EXPECT_NEAR(0.0, ekf_.X_hat()(1), 1e-10);
}
}  // namespace frc971::control_loops::testing
