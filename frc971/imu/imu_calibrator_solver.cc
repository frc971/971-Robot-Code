#include "frc971/imu/imu_calibrator_solver.h"
namespace frc971::imu {

struct ImuCalibratorCostFunctor {
  ImuCalibratorCostFunctor(
      const std::vector<std::vector<RawImuReading>> &readings,
      const std::vector<ImuConfig<double>> &nominal_config,
      const size_t num_residuals)
      : readings_(readings),
        nominal_config_(nominal_config),
        num_residuals_(num_residuals) {}
  template <typename S>
  bool operator()(const S *const *const parameters_ptr, S *residual) const {
    AllParameters<S> params = AllParameters<S>::PopulateFromParameters(
        nominal_config_, parameters_ptr);
    std::vector<ImuConfig<S>> imu_configs;
    for (const auto &param : params.imus) {
      imu_configs.push_back(param);
    }
    ImuCalibrator<S> calibrator(imu_configs);
    for (size_t imu_index = 0; imu_index < readings_.size(); ++imu_index) {
      const auto imu_readings = readings_[imu_index];
      for (size_t reading_index = 0; reading_index < imu_readings.size();
           ++reading_index) {
        calibrator.InsertImu(imu_index, imu_readings[reading_index]);
      }
    }
    calibrator.CalculateResiduals({residual, num_residuals_});
    return true;
  }
  const std::vector<std::vector<RawImuReading>> readings_;
  const std::vector<ImuConfig<double>> nominal_config_;
  const size_t num_residuals_;
};

AllParameters<double> Solve(
    const std::vector<std::vector<RawImuReading>> &readings,
    const std::vector<ImuConfig<double>> &nominal_config) {
  ceres::Problem problem;

  ceres::EigenQuaternionManifold *quaternion_local_parameterization =
      new ceres::EigenQuaternionManifold();
  AllParameters<double> parameters;
  std::vector<size_t> num_readings;
  CHECK_EQ(nominal_config.size(), readings.size());
  for (size_t imu_index = 0; imu_index < nominal_config.size(); ++imu_index) {
    const size_t num_params = readings[imu_index].size();
    parameters.imus.emplace_back(nominal_config[imu_index]);
    num_readings.push_back(num_params);
  }
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

  {
    const size_t num_residuals =
        ImuCalibrator<double>::CalculateNumResiduals(num_readings);
    ceres::DynamicCostFunction *cost_function =
        new ceres::DynamicAutoDiffCostFunction<ImuCalibratorCostFunctor>(
            new ImuCalibratorCostFunctor(readings, nominal_config,
                                         num_residuals));

    auto [vector_parameters, post_populate_methods] =
        parameters.PopulateParameters(quaternion_local_parameterization,
                                      &problem, cost_function);

    cost_function->SetNumResiduals(num_residuals);

    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                             vector_parameters);
    for (auto &method : post_populate_methods) {
      method();
    }
  }

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-12;
  options.function_tolerance = 1e-6;
  options.parameter_tolerance = 1e-6;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport();
  LOG(INFO) << "Solution is " << (summary.IsSolutionUsable() ? "" : "NOT ")
            << "usable";
  LOG(INFO) << "Solution:\n" << parameters.ToString();
  return parameters;
}
}  // namespace frc971::imu
