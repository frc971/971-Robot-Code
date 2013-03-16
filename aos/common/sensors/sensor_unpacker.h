#ifndef AOS_COMMON_SENSORS_SENSOR_UNPACKER_H_
#define AOS_COMMON_SENSORS_SENSOR_UNPACKER_H_

namespace aos {
namespace sensors {

// An interface that handles taking data from the sensor Values struct and
// putting it into queues (for control loops etc).
// See sensors.h for an overview of where this fits in.
template<class Values>
class SensorUnpackerInterface {
 public:
  virtual ~SensorUnpackerInterface() {}

  // Takes the data in *values and writes it out into queues etc.
  virtual void UnpackFrom(Values *values) = 0;
};

}  // namespace sensors
}  // namespace aos

#endif  // AOS_COMMON_SENSORS_SENSOR_UNPACKER_H_
