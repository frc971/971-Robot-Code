#ifndef FRC971_IMU_IMU_CALIBRATOR_SOLVER_H_
#define FRC971_IMU_IMU_CALIBRATOR_SOLVER_H_

#include "frc971/imu/imu_calibrator.h"

namespace frc971::imu {

// Stores all the IMU data from a log so that we can feed it into the
// ImuCalibrator readily.
AllParameters<double> Solve(
    const std::vector<std::vector<RawImuReading>> &readings,
    const std::vector<ImuConfig<double>> &nominal_config);
}  // namespace frc971::imu
#endif  // FRC971_IMU_IMU_CALIBRATOR_SOLVER_H_
