#include "frc971/control_loops/swerve/velocity_ekf.h"

#include <random>

#include "gtest/gtest.h"

#include "aos/flatbuffers/builder.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/math/eigen_matchers.h"

namespace frc971::control_loops::swerve::testing {
class VelocityEkfTest : public ::testing::Test {
 protected:
  typedef double Scalar;
  using Dynamics = SimplifiedDynamics<Scalar>;
  using States = Dynamics::States;
  using State = Dynamics::VelocityState<Scalar>;
  using Inputs = Dynamics::Inputs;
  using Input = Dynamics::Input<Scalar>;
  using Ekf = VelocityEkf<Scalar>;
  using ModuleParams = Dynamics::ModuleParams;
  using Parameters = Dynamics::Parameters;
  static ModuleParams MakeModule(const Eigen::Matrix<Scalar, 2, 1> &position) {
    return ModuleParams{.position = position,
                        .slip_angle_coefficient = 200.0,
                        .slip_angle_alignment_coefficient = 0.0,
                        .steer_motor = KrakenFOC(),
                        .drive_motor = KrakenFOC(),
                        .steer_ratio = 0.1,
                        .drive_ratio = 0.01,
                        .wheel_radius = 1.0,
                        .extra_steer_inertia = 0.0};
  }
  static Parameters MakeParams() {
    return {.mass = 60,
            .moment_of_inertia = 2,
            .modules =
                {
                    MakeModule({1.0, 1.0}),
                    MakeModule({-1.0, 1.0}),
                    MakeModule({-1.0, -1.0}),
                    MakeModule({1.0, -1.0}),
                },
            .accel_weight = 0.0};
  }

  VelocityEkfTest()
      : dynamics_(MakeParams()),
        expected_drive_velocity_(MakeParams()),
        true_state_(State::Zero()),
        ekf_(MakeParams()) {
    ekf_.Initialize(now_, true_state_);
    drive_encoder_times_.fill(aos::monotonic_clock::epoch());
    drive_encoders_.setZero();
  }

  void IntegrateDynamics(const Input &U, std::chrono::nanoseconds dt) {
    now_ += dt;
    true_state_ = RungeKuttaU(dynamics_, true_state_, U,
                              aos::time::DurationInSeconds(dt));
    const Eigen::Matrix<Scalar, 4, 1> drive_velocities =
        expected_drive_velocity_(true_state_);
    drive_encoder_times_.fill(now_);
    // Note: This does not actually do an ideal integration of the drive
    // encoders.
    drive_encoders_ += drive_velocities * aos::time::DurationInSeconds(dt);
  }

  Eigen::Matrix<Scalar, 4, 1> GetSteerEncoders() const {
    return Eigen::Matrix<Scalar, 4, 1>{{true_state_(States::kThetas0)},
                                       {true_state_(States::kThetas1)},
                                       {true_state_(States::kThetas2)},
                                       {true_state_(States::kThetas3)}};
  }

  Scalar GetYawRate() const { return true_state_(States::kOmega); }

  enum class DoNoisyUpdate { kYes, kNo };

  void DoStep(const Input &U, std::chrono::nanoseconds dt,
              DoNoisyUpdate noisy_update = DoNoisyUpdate::kNo) {
    IntegrateDynamics(U, dt);
    Eigen::Matrix<Scalar, 4, 1> steer_encoders = GetSteerEncoders();
    Eigen::Matrix<Scalar, 4, 1> drive_encoders = drive_encoders_;
    double yaw_rate = GetYawRate();
    if (noisy_update == DoNoisyUpdate::kYes) {
      // Eigen's Random() provides numbers in the range [-1, 1]; we scale them
      // to "reasonable" levels of noise here.
      steer_encoders += GaussianRandomVector<4>(0.00001);
      drive_encoders += GaussianRandomVector<4>(0.00001);
      yaw_rate += GaussianRandomVector<1>(1e-3)(0);
    }
    aos::fbs::Builder<VelocityEkfStatusStatic> status_builder;
    ekf_.Update(now_, steer_encoders, drive_encoder_times_, drive_encoders,
                yaw_rate, U, status_builder.get());
  }

  template <int N>
  Eigen::Matrix<Scalar, N, 1> GaussianRandomVector(double standard_deviation) {
    std::normal_distribution distribution{0.0, standard_deviation};
    Eigen::Matrix<Scalar, N, 1> random;
    for (int index = 0; index < N; ++index) {
      random(index) = distribution(random_engine_);
    }
    return random;
  }

  Dynamics::VirtualVelocityDynamics dynamics_;
  aos::monotonic_clock::time_point now_ = aos::monotonic_clock::epoch();
  Ekf::ExpectedDriveVelocity expected_drive_velocity_;
  std::array<aos::monotonic_clock::time_point, 4> drive_encoder_times_;
  Eigen::Matrix<Scalar, 4, 1> drive_encoders_;
  State true_state_;
  Ekf ekf_;

  std::mt19937 random_engine_;
};

// Validates that if we have perfect corrections that we perfectly match the
// underlying dynamics.
TEST_F(VelocityEkfTest, PerfectCorrections) {
  constexpr std::chrono::milliseconds kDt{1};
  for (int i = 0; i < 1000; ++i) {
    DoStep(Input::Zero(), kDt);
    EXPECT_EQ(true_state_, ekf_.X_hat());
    EXPECT_EQ(0.0, ekf_.X_hat().norm());
  }
  for (int i = 0; i < 1000; ++i) {
    DoStep(Input::Ones(), kDt);
    // Once we start moving we can get some numerical errors starting to
    // accumulate, so use a threshold.
    EXPECT_THAT(ekf_.X_hat(),
                frc971::math::IsEigenMatrixNear(true_state_, 1e-10));
    // Sanity check that the velocity of the robot has actually increased.
    EXPECT_LT(0.0, ekf_.X_hat()(States::kVx));
  }
}

// Tests that if we get corrections with small amounts of noise that the
// corrections still behave reasonably.
TEST_F(VelocityEkfTest, NoisyCorrections) {
  constexpr std::chrono::milliseconds kDt{1};
  for (int i = 0; i < 1000; ++i) {
    DoStep(Input::Zero(), kDt, DoNoisyUpdate::kYes);
    EXPECT_THAT(true_state_,
                frc971::math::IsEigenMatrixNear(ekf_.X_hat(), 1e-1));
  }
  for (int i = 0; i < 1000; ++i) {
    DoStep(Input::Ones(), kDt, DoNoisyUpdate::kYes);
    State threshold = State::Ones() * 1e-1;
    // Because we have no absolute sensor reading for theta, it will drift over
    // time; all other states should be kept near their true values.
    threshold(States::kTheta) = 1.0;
    EXPECT_THAT(true_state_, frc971::math::AreEigenMatrixElementsNear(
                                 ekf_.X_hat(), threshold));
  }
}

}  // namespace frc971::control_loops::swerve::testing
