#include "frc971/imu/imu_calibrator.h"

ABSL_FLAG(
    int32_t, imu_zeroing_buffer, 100,
    "We will only consider readings stationary for purposes if calibration if "
    "this many readings to either side appear to be stationary.");
