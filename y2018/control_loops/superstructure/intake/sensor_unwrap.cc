#include "y2018/control_loops/superstructure/intake/sensor_unwrap.h"

#include <cmath>

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace intake {

UnwrapSensor::UnwrapSensor(double sensor_offset, double sensor_range)
    : sensor_offset_(sensor_offset), sensor_range_(sensor_range) {
  Reset();
}

double UnwrapSensor::Unwrap(double current_sensor_value) {
  // First time the function is called it will use that value to initialize the
  // wrap calculation. This catches cases where the offset and first value
  // difference triggers an unwanted wrap at the first calculation.
  if (uninitialized_ == true) {
    uninitialized_ = false;
  } else {
    // Calculates the lower sensor value and set the max sensor range
    // If offset is not 0, this will correct the zeroing offset
    const double sensor_min_value = sensor_offset_;
    const double sensor_max_value = sensor_range_ + sensor_min_value;

    // Check if provided sensor value is within the range. This to prevent the
    // function to get out of sync. Will not throw an error, but continue and
    // return the value + wrapped factor and not process this value.
    if (current_sensor_value < sensor_min_value ||
        current_sensor_value > sensor_max_value) {
      return current_sensor_value + (sensor_range_ * wrap_count_);
    }

    // Calculate the positive or negative movement
    const double sensor_move = current_sensor_value - sensor_last_value_;

    // Function assumes that a movement of more then 1/2 of the range
    // indicates that we wrapped, instead of moved very fast.
    if (std::abs(sensor_move) > (sensor_range_ / 2)) {
      if (sensor_move >= 0) {
        // sensor moved past the sensor_min_value
        wrap_count_ -= 1;
      } else {
        // sensor moved past the sensor_max_value
        wrap_count_ += 1;
      }
    }
  }
  sensor_last_value_ = current_sensor_value;
  // return the unwrapped sensor value
  return current_sensor_value + (sensor_range_ * wrap_count_);
}

void UnwrapSensor::Reset() {
  wrap_count_ = 0;
  sensor_last_value_ = sensor_offset_;
  uninitialized_ = true;
}

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
