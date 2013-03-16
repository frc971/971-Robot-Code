#ifndef FRC971_INPUT_SENSOR_UNPACKER_H_
#define FRC971_INPUT_SENSOR_UNPACKER_H_

#include "aos/common/sensors/sensor_unpacker.h"

#include "frc971/queues/sensor_values.h"

namespace frc971 {

class SensorUnpacker
    : public ::aos::sensors::SensorUnpackerInterface<sensor_values> {
 public:
  SensorUnpacker();

  virtual void UnpackFrom(sensor_values *values);
};

}  // namespace frc971

#endif  // FRC971_INPUT_SENSOR_UNPACKER_H_
