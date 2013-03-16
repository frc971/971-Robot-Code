#ifndef AOS_COMMON_SENSORS_SENSOR_SINK_H_
#define AOS_COMMON_SENSORS_SENSOR_SINK_H_

#include "aos/common/sensors/sensors.h"

namespace aos {
namespace sensors {

// Generic class for something that can do something with sensor data.
template<class Values>
class SensorSinkInterface {
 public:
  virtual ~SensorSinkInterface() {}

  virtual void Process(SensorData<Values> *data) = 0;
};

}  // namespace sensors
}  // namespace aos

#endif  // AOS_COMMON_SENSORS_SENSOR_SINK_H_
