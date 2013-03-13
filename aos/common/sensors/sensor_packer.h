#ifndef AOS_COMMON_SENSORS_SENSOR_PACKER_H_
#define AOS_COMMON_SENSORS_SENSOR_PACKER_H_

namespace aos {
namespace sensors {

// An interface that handles reading input data and putting it into the sensor
// values struct.
// See sensors.h for an overview of where this fits in.
template<class Values>
class SensorPackerInterface {
 public:
  virtual ~SensorPackerInterface() {}

  // Reads the inputs (from WPILib etc) and writes the data into *values.
  virtual void PackInto(Values *values) = 0;
};

}  // namespace sensors
}  // namespace aos

#endif  // AOS_COMMON_SENSORS_SENSOR_PACKER_H_
