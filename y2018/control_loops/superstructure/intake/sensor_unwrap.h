#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_SENSOR_UNWRAP_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_SENSOR_UNWRAP_H_

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace intake {

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

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_SENSOR_UNWRAP_H_
