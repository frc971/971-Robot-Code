#include "frc971/imu/imu_calibrator.h"

#include <random>

#include "absl/flags/reflection.h"
#include "gtest/gtest.h"

#include "frc971/imu/imu_calibrator_solver.h"

namespace frc971::imu::testing {
class ImuSimulator {
 public:
  ImuSimulator(const std::vector<ImuConfig<double>> &imu_configs)
      : imu_configs_(imu_configs), readings_(imu_configs.size()) {}
  void SimulateForTime(aos::monotonic_clock::duration duration,
                       aos::monotonic_clock::duration dt,
                       const Eigen::Vector3d &gravity_vector,
                       const Eigen::Vector3d &accel,
                       const Eigen::Vector3d &gyro) {
    for (const aos::monotonic_clock::time_point end_time = now + duration;
         now < end_time; now += dt) {
      for (size_t imu_index = 0; imu_index < imu_configs_.size(); ++imu_index) {
        const ImuConfig<double> &config = imu_configs_[imu_index];
        const Eigen::Quaterniond rotation =
            config.is_origin ? Eigen::Quaterniond::Identity()
                             : config.parameters->rotation.inverse();
        const std::chrono::nanoseconds time_offset{
            config.is_origin
                ? 0
                : static_cast<uint64_t>(config.parameters->time_offset * 1e9)};
        readings_[imu_index].emplace_back(
            now + time_offset,
            rotation * gyro + config.dynamic_params.gyro_zero + GyroNoise(),
            rotation *
                    (accel + gravity_vector * config.dynamic_params.gravity) +
                AccelNoise());
      }
    }
  }
  Eigen::Vector3d GyroNoise() {
    return (enable_noise_ ? 1.0 : 0.0) *
           Eigen::Vector3d(distribution_(generator_), distribution_(generator_),
                           distribution_(generator_));
  }
  Eigen::Vector3d AccelNoise() { return GyroNoise() * 2.0; }
  void set_enable_noise(bool enable) { enable_noise_ = enable; }
  std::vector<std::vector<RawImuReading>> readings() const {
    return readings_;
  };

 private:
  const std::vector<ImuConfig<double>> imu_configs_;
  std::vector<std::vector<RawImuReading>> readings_;
  aos::monotonic_clock::time_point now = aos::monotonic_clock::epoch();

  std::mt19937 generator_;
  std::normal_distribution<> distribution_{0.0, 0.00025};
  bool enable_noise_ = false;
};

namespace {
void VerifyParameters(const std::vector<ImuConfig<double>> &imus,
                      const AllParameters<double> &params, double eps = 1e-8) {
  ASSERT_EQ(imus.size(), params.imus.size());
  for (size_t imu_index = 0; imu_index < imus.size(); ++imu_index) {
    SCOPED_TRACE(imu_index);
    const ImuConfig<double> &expected = imus[imu_index];
    const ImuConfig<double> &calculated = params.imus[imu_index];
    EXPECT_EQ(expected.parameters.has_value(),
              calculated.parameters.has_value());
    EXPECT_NEAR(expected.dynamic_params.gravity,
                calculated.dynamic_params.gravity, eps);
    EXPECT_LT((expected.dynamic_params.gyro_zero -
               calculated.dynamic_params.gyro_zero)
                  .norm(),
              eps)
        << expected.dynamic_params.gyro_zero.transpose() << " vs. "
        << calculated.dynamic_params.gyro_zero.transpose();
    if (expected.parameters.has_value()) {
      EXPECT_NEAR(expected.parameters->time_offset,
                  calculated.parameters->time_offset, eps);
      EXPECT_LT(((expected.parameters->rotation *
                  calculated.parameters->rotation.inverse())
                     .coeffs() -
                 Eigen::Quaterniond::Identity().coeffs())
                    .norm(),
                eps)
          << expected.parameters->rotation.coeffs().transpose() << " vs. "
          << calculated.parameters->rotation.coeffs().transpose();
    }
  }
}
}  // namespace

// Confirms that we can calibrate in a relatively simple scenario where we have
// some gyro/accelerometer offsets and a small rotation that is not accounted
// for in the nominal parameters.
TEST(ImuCalibratorTest, BasicCalibrationTest) {
  std::vector<ImuConfig<double>> nominal_imus = {
      ImuConfig<double>{true, std::nullopt},
      ImuConfig<double>{false, std::make_optional<StaticImuParameters<double>>(
                                   Eigen::Quaterniond::Identity(), 0.0)}};

  std::vector<ImuConfig<double>> real_imus = nominal_imus;
  real_imus[0].dynamic_params.gravity = 1.005;
  real_imus[0].dynamic_params.gyro_zero << 0.001, 0.002, 0.003;
  real_imus[1].dynamic_params.gyro_zero << -0.009, -0.007, -0.001;
  real_imus[1].parameters->rotation =
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ());
  ImuSimulator simulator(real_imus);
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  simulator.SimulateForTime(
      std::chrono::seconds(1), std::chrono::milliseconds(1),
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Vector3d(1.0, 0.0, 0.0));
  simulator.SimulateForTime(
      std::chrono::seconds(1), std::chrono::milliseconds(1),
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Vector3d(0.0, 1.0, 0.0));
  auto params = Solve(simulator.readings(), nominal_imus);
  LOG(INFO) << params.ToString();
  LOG(INFO) << real_imus[0].ToString();
  LOG(INFO) << real_imus[1].ToString();
  VerifyParameters(real_imus, params);
}

// Separately test the estimation of the time offset between IMUs.
// This is done separately because the optimization problem is poorly condition
// for estimating time offsets when there is just a handful of step changes in
// the IMU inputs; feeding in a sine wave works much better for allowing the
// solver to estimate the offset.
TEST(ImuCalibratorTest, TimeOffsetTest) {
  absl::FlagSaver flag_saver;

  std::vector<ImuConfig<double>> nominal_imus = {
      ImuConfig<double>{true, std::nullopt},
      ImuConfig<double>{false, std::make_optional<StaticImuParameters<double>>(
                                   Eigen::Quaterniond::Identity(), 0.0)}};

  std::vector<ImuConfig<double>> real_imus = nominal_imus;
  real_imus[1].parameters->time_offset = 0.0255;
  ImuSimulator simulator(real_imus);
  // Note on convergence: It is easy to end up in situations where the problem
  // is not outstandingly well conditioned and we can end up with local minima
  // where changes to physical calibration attributes can explain the time
  // offset.
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  for (size_t ii = 0; ii < 10000; ++ii) {
    simulator.SimulateForTime(
        std::chrono::milliseconds(1), std::chrono::milliseconds(1),
        Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(std::sin(ii / 1000.0), 0.0, 0.0));
  }
  auto params = Solve(simulator.readings(), nominal_imus);
  LOG(INFO) << params.ToString();
  LOG(INFO) << real_imus[0].ToString();
  LOG(INFO) << real_imus[1].ToString();
  VerifyParameters(real_imus, params, 1e-6);
}

// Test that if we add in some random noise that the solver still behaves
// itself.
TEST(ImuCalibratorTest, RandomNoise) {
  std::vector<ImuConfig<double>> nominal_imus = {
      ImuConfig<double>{true, std::nullopt},
      ImuConfig<double>{false, std::make_optional<StaticImuParameters<double>>(
                                   Eigen::Quaterniond::Identity(), 0.0)}};

  std::vector<ImuConfig<double>> real_imus = nominal_imus;
  real_imus[0].dynamic_params.gravity = 0.999;
  real_imus[0].dynamic_params.gyro_zero << 0.001, 0.002, 0.003;
  real_imus[1].dynamic_params.gyro_zero << -0.009, -0.007, -0.001;
  real_imus[1].parameters->rotation =
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ());
  ImuSimulator simulator(real_imus);
  simulator.set_enable_noise(true);
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  simulator.SimulateForTime(std::chrono::seconds(1),
                            std::chrono::milliseconds(1),
                            Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0),
                            Eigen::Vector3d(0.0, 0.0, 0.0));
  for (size_t ii = 0; ii < 6000; ++ii) {
    simulator.SimulateForTime(std::chrono::milliseconds(1),
                              std::chrono::milliseconds(1),
                              Eigen::Vector3d(0, 0, 1),
                              Eigen::Vector3d(std::sin(ii / 1000.0), 0.0, 0.0),
                              Eigen::Vector3d(1.0, 0.0, 0.0));
  }
  auto params = Solve(simulator.readings(), nominal_imus);
  LOG(INFO) << params.ToString();
  LOG(INFO) << real_imus[0].ToString();
  LOG(INFO) << real_imus[1].ToString();
  VerifyParameters(real_imus, params, 1e-4);
}
}  // namespace frc971::imu::testing
