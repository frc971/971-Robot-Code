#ifndef FRC971_IMU_IMU_CALIBRATOR_H_
#define FRC971_IMU_IMU_CALIBRATOR_H_

#include <optional>
#include <span>
#include <tuple>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aos/time/time.h"

namespace frc971::imu {

// Contains a reading that comes directly from an IMU.
// These should not be zeroed or corrected for yet.
struct RawImuReading {
  aos::monotonic_clock::time_point capture_time;
  // gyro readings are in radians / sec; accelerometer readings are in g's.
  const Eigen::Vector3d gyro;
  const Eigen::Vector3d accel;
};

// Note on how we manage populating ceres parameters:
// * We use dynamically-sized parameter lists for convenience.
// * For every struct that we have that corresponds to problem parameters,
//   there is a PopulateParameters() method that takes a:
//   * ceres::Problem* that is used for setting parameter constraints.
//   * ceres::DynamicCostFunction* that is used to mark that we have added
//     a parameter block.
//   * A std::vector<double*> that will later be passed to AddResidualBlock.
//     We add any parameter blocks which we add to the problem.
//   * post_populate_methods is a list of std::function's that will get called
//     after everything is populated and added (this is necessary because
//     we cannot add constraints to the Problem until after everything
//     has been added).
// * Additionally, there is a PopulateFromParameters() method which is used
//   in the ceres cost functor and:
//   * Takes a parameters double-pointer where parameters[X] is the
//     parameter block X.
//   * Returns a pair where the first value is a populated instance of the
//     struct and an updated parameters pointer which points to the next
//     set of pointers.
// * All the Populate* methods must be called in the same order every
//   time so that they result in the raw pointers getting populated
//   consistently.

// These are the parameters corresponding to things which will vary at every
// power-on of the IMU.
template <typename Scalar>
struct DynamicImuParameters {
  // A scalar to represent the current magnitude of gravity, in g's.
  // This currently compensates both for any variations in local gravity as well
  // as for some amount of variations in the IMU itself. In the future we may
  // expand this to have a local gravity number that is global to all IMUs while
  // separately calibrating per-axis information.
  Scalar gravity;
  // Current gyro zero, in radians / sec.
  // These are the per-axis values that the gyro will read when it is sitting
  // still. Technically these zeroes will drift over time; however, the
  // time-windows that we expect to use for calibration are short enough that
  // this should be a non-issue.
  Eigen::Matrix<Scalar, 3, 1> gyro_zero;
  void PopulateParameters(
      ceres::Problem *problem, ceres::DynamicCostFunction *cost_function,
      std::vector<double *> *parameters,
      std::vector<std::function<void()>> *post_populate_methods) {
    cost_function->AddParameterBlock(1);
    parameters->push_back(&gravity);
    cost_function->AddParameterBlock(3);
    parameters->push_back(gyro_zero.data());
    post_populate_methods->emplace_back([this, problem]() {
      // Gravity shouldn't vary by much (these bounds are significantly larger
      // than any real variations which we will experience).
      problem->SetParameterLowerBound(&gravity, 0, 0.95);
      problem->SetParameterUpperBound(&gravity, 0, 1.05);
      for (int i = 0; i < 3; ++i) {
        problem->SetParameterLowerBound(gyro_zero.data(), i, -0.05);
        problem->SetParameterUpperBound(gyro_zero.data(), i, 0.05);
      }
    });
  }
  static std::tuple<DynamicImuParameters<Scalar>, const Scalar *const *>
  PopulateFromParameters(const Scalar *const *parameters) {
    const Scalar *const gravity = parameters[0];
    ++parameters;
    const Scalar *const gyro = parameters[0];
    ++parameters;
    return std::make_tuple(
        DynamicImuParameters<Scalar>{
            *gravity, Eigen::Matrix<Scalar, 3, 1>(gyro[0], gyro[1], gyro[2])},
        parameters);
  }
  std::string ToString() const {
    std::stringstream out;
    out << "gravity: " << gravity << " gyro_zero: " << gyro_zero.transpose();
    return out.str();
  }
};

// These parameters correspond to IMU parameters which will not vary between
// boots. Namely, the relative positions and time offsets of the IMU(s).
template <typename Scalar>
struct StaticImuParameters {
  // If you have a point p_imu in this IMU's frame then (rotation *
  // p_imu + position) will give you that point's position in the board frame
  // (the "board" frame refers to the frame associated with the entire PCB,
  // where the PCB itself contains multiple IMUs. The board frame will be
  // attached to whichever IMU is being treated as the origin).
  Eigen::Quaternion<Scalar> rotation;
  // position is currently unused because it is only observeable when there
  // are coriolis effects, and we currently only make use of accelerometer
  // readings from when the IMU is sat still.
  // As such, we currently assume that all position offsets are zero.
  // Eigen::Matrix<Scalar, 3, 1> position;
  // The "true" time at which the event occurred is equal to the capture time -
  // time_offset. I.e., a more positive time offset implies that the there is a
  // large delay between the imu readings being taken and us observing them.
  Scalar time_offset;

  void PopulateParameters(
      ceres::EigenQuaternionManifold *quaternion_local_parameterization,
      ceres::Problem *problem, ceres::DynamicCostFunction *cost_function,
      std::vector<double *> *parameters,
      std::vector<std::function<void()>> *post_populate_methods) {
    cost_function->AddParameterBlock(4);
    parameters->push_back(rotation.coeffs().data());
    cost_function->AddParameterBlock(1);
    parameters->push_back(&time_offset);
    post_populate_methods->emplace_back(
        [this, problem, quaternion_local_parameterization]() {
          problem->SetManifold(rotation.coeffs().data(),
                               quaternion_local_parameterization);
          problem->SetParameterLowerBound(&time_offset, 0, -0.03);
          problem->SetParameterUpperBound(&time_offset, 0, 0.03);
        });
  }
  static std::tuple<StaticImuParameters, const Scalar *const *>
  PopulateFromParameters(const Scalar *const *parameters) {
    const Scalar *const quat = parameters[0];
    ++parameters;
    const Scalar *const time = parameters[0];
    ++parameters;
    return std::make_tuple(
        StaticImuParameters{
            Eigen::Quaternion<Scalar>(quat[3], quat[0], quat[1], quat[2]),
            *time},
        parameters);
  }
  std::string ToString() const {
    std::stringstream out;
    out << "quat: " << rotation.coeffs().transpose()
        << " time_offset: " << time_offset;
    return out.str();
  }
};

// Represents the calibration for a single IMU.
template <typename Scalar>
struct ImuConfig {
  // Set to true if this IMU is to be considered the origin of the coordinate
  // system. This will also mean that this IMU is treated as the source-of-truth
  // for clock offsets.
  bool is_origin;
  // Will be nullopt if is_origin is true (the corresponding rotations and
  // offsets will all be the identity/zero as appropriate).
  std::optional<StaticImuParameters<Scalar>> parameters;

  DynamicImuParameters<Scalar> dynamic_params{
      .gravity = static_cast<Scalar>(1.0),
      .gyro_zero = Eigen::Matrix<Scalar, 3, 1>::Zero()};
  std::string ToString() const {
    return absl::StrFormat(
        "is_origin: %d params: %s dynamic: %s", is_origin,
        parameters.has_value() ? parameters->ToString() : std::string("<null>"),
        dynamic_params.ToString());
  }
};

// Represents all of the configuration parameters for the entire system.
template <typename Scalar>
struct AllParameters {
  // Each entry in the imus vector will be a single imu.
  std::vector<ImuConfig<Scalar>> imus;
  std::tuple<std::vector<double *>, std::vector<std::function<void()>>>
  PopulateParameters(
      ceres::EigenQuaternionManifold *quaternion_local_parameterization,
      ceres::Problem *problem, ceres::DynamicCostFunction *cost_function) {
    std::vector<std::function<void()>> post_populate_methods;
    std::vector<double *> parameters;
    for (auto &imu : imus) {
      if (imu.parameters.has_value()) {
        imu.parameters.value().PopulateParameters(
            quaternion_local_parameterization, problem, cost_function,
            &parameters, &post_populate_methods);
      }
      imu.dynamic_params.PopulateParameters(problem, cost_function, &parameters,
                                            &post_populate_methods);
    }
    return std::make_tuple(parameters, post_populate_methods);
  }
  static AllParameters PopulateFromParameters(
      const std::vector<ImuConfig<double>> &nominal_configs,
      const Scalar *const *parameters) {
    std::vector<ImuConfig<Scalar>> result;
    for (size_t imu_index = 0; imu_index < nominal_configs.size();
         ++imu_index) {
      ImuConfig<Scalar> config;
      config.is_origin = nominal_configs[imu_index].is_origin;
      if (!config.is_origin) {
        std::tie(config.parameters, parameters) =
            StaticImuParameters<Scalar>::PopulateFromParameters(parameters);
      }
      std::tie(config.dynamic_params, parameters) =
          DynamicImuParameters<Scalar>::PopulateFromParameters(parameters);
      result.emplace_back(std::move(config));
    }
    return {.imus = std::move(result)};
  }
  std::string ToString() const {
    std::vector<std::string> imu_strings;
    for (const auto &imu : imus) {
      std::vector<std::string> dynamic_params;
      imu_strings.push_back(absl::StrFormat("config: %s", imu.ToString()));
    }
    return absl::StrJoin(imu_strings, "\n");
  }
};

// This class does the hard work to support calibrating multiple IMU's
// orientations relative to one another. It is set up to readily be used with a
// ceres solver (see imu_calibrator.*).
//
// The current theory of operation is to have some number of imus, one of which
// we will consider to be fixed in position. We have a fixed set of data that we
// feed into this class, which can be parameterized based on:
// * The current zeroes for each IMU.
// * The orientation of each non-fixed IMU.
// * The time-offset of each non-fixed IMU.
//
// When run under ceres, ceres can then vary these parameters to solve for the
// current calibrations of the IMUs.
//
// When solving, the main complexity is that some internal state has to be
// tracked to try to determine when we should be calculating zeroes and when we
// can calibrate out the magnitude of gravity. This is done by tracking when the
// IMUs are "stationary". For a reading to be stationary, all values with
// FLAGS_imu_zeroing_buffer readings of the reading must be "plausibly
// stationary". Readings are plausibly stationary if they have sufficiently low
// gyro values.
//
// TODO: Provide some utilities to plot the results of a calibration.
template <typename Scalar>
class ImuCalibrator {
 public:
  ImuCalibrator(const std::vector<ImuConfig<Scalar>> &imu_configs)
      : imu_configs_(imu_configs), imu_readings_(imu_configs.size()) {
    origin_index_ = -1;
    for (size_t imu_index = 0; imu_index < imu_configs_.size(); ++imu_index) {
      if (imu_configs_[imu_index].is_origin) {
        CHECK_EQ(origin_index_, -1)
            << ": Can't have more than one IMU specified as the origin.";
        origin_index_ = imu_index;
      }
    }
    CHECK_NE(origin_index_, -1)
        << ": Must have at least one IMU specified as the origin.";
  }

  // These gyro readings will be "raw"---i.e., they still need to get
  // transformed by nominal_transform.
  // gyro readings are in radians / sec; accelerometer readings are in g's.
  // Within a given imu, must be called in time order.
  void InsertImu(size_t imu_index, const RawImuReading &reading);

  // Populates all the residuals that we use for the cost in ceres.
  void CalculateResiduals(std::span<Scalar> residuals);

  // Returns the total number of residuals that this problem will have, given
  // the number of readings for each IMU.
  static size_t CalculateNumResiduals(const std::vector<size_t> &num_readings);

 private:
  // Represents an imu reading. The values in this are generally already
  // adjusted for the provided parameters.
  struct ImuReading {
    // The "actual" provided capture time. Used for debugging.
    aos::monotonic_clock::time_point capture_time_raw;
    // The capture time, adjusted for this IMU's time offset.
    Scalar capture_time_adjusted;
    // gyro reading, adjusted for gyro zero but NOT rotation.
    // In radians / sec.
    Eigen::Matrix<Scalar, 3, 1> gyro;
    // accelerometer reading, adjusted for gravity value but NOT rotation.
    // In g's.
    Eigen::Matrix<Scalar, 3, 1> accel;
    // Residuals associated with the DynamicImuParameters for this imu.
    DynamicImuParameters<Scalar> parameter_residuals;
    // Set if this measurement *could* be part of a segment of time where the
    // IMU is stationary.
    bool plausibly_stationary;
    // Set to true if all values with FLAGS_imu_zeroing_buffer of this reading
    // are plausibly_stationary.
    bool stationary;
    // We set stationary_is_locked once we have enough readings to either side
    // of this value to guarantee that it is stationary.
    bool stationary_is_locked;
    // Residuals that are used for calibrating the rotation values. These are
    // nullopt if we can't calibrate for some reason (e.g., this is the origin
    // IMU, or for the accelerometer residual, we don't populate it if we are
    // not stationary).
    std::optional<Eigen::Matrix<Scalar, 3, 1>> gyro_residual;
    std::optional<Eigen::Matrix<Scalar, 3, 1>> accel_residual;
  };
  void EvaluateRelativeResiduals();

  const std::vector<ImuConfig<Scalar>> imu_configs_;
  // Index of the IMU which is the origin IMU.
  int origin_index_;
  std::vector<std::vector<ImuReading>> imu_readings_;
};

}  // namespace frc971::imu

#include "frc971/imu/imu_calibrator-tmpl.h"
#endif  // FRC971_IMU_IMU_CALIBRATOR_H_
