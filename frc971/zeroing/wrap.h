#ifndef FRC971_ZEROING_WRAP_H_
#define FRC971_ZEROING_WRAP_H_

namespace frc971 {
namespace zeroing {

// UnwrapSensor takes in a sensor value from a sensor that loops in a certain
// interval. ex(the sensor moves from 0 to 10 and back to 0 while moving the
// same direction) By checking for big gaps in sensor readings it assumes you
// have wrapped either back or forwards and handles accordingly. It returns the
// overall sensor value.

class UnwrapSensor {
 public:
  // The sensor_offset (+ or -) present the sensor value that is 'zero'
  // The sensor_range presents the absolute value of the sensor range from 0 to
  // sensor_range. This will be adjusted using the sensor_offset
  UnwrapSensor(double sensor_offset, double sensor_range);

  // Takes a wrapped sensor value and unwraps it to give you its total position.
  double Unwrap(double current_sensor_value);

  void Reset();

  int sensor_wrapped() const { return wrap_count_; }

 private:
  const double sensor_offset_, sensor_range_;

  // The last value given from set_position, starts at offset
  double sensor_last_value_ = sensor_offset_;

  // Log if sensor is in wrapped state in either direction
  int wrap_count_ = 0;

  // function waits for first call with a value to set sensor_last_value_. Will
  // start to calculate the spring unwrap at the second function call.
  bool uninitialized_ = true;
};

// Returns a modified value which has been wrapped such that it is +- period/2
// away from nearest.
double Wrap(double nearest, double value, double period);
float Wrap(float nearest, float value, float period);

inline double UnWrap(double nearest, double value, double period) {
  return Wrap(nearest, value, period);
}
inline float UnWrap(float nearest, float value, float period) {
  return Wrap(nearest, value, period);
}

}  // namespace zeroing
}  // namespace frc971

#endif  // FRC971_ZEROING_WRAP_H_
