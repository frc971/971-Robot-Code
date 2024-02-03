#include "frc971/imu/imu_calibrator.h"

DEFINE_int32(
    imu_zeroing_buffer, 100,
    "We will only consider readings stationary for purposes if calibration if "
    "this many readings to either side appear to be stationary.");
