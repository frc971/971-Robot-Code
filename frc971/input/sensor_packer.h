#ifndef FRC971_INPUT_SENSOR_PACKER_H_
#define FRC971_INPUT_SENSOR_PACKER_H_

#include "aos/common/mutex.h"
#include "aos/crio/shared_libs/interrupt_notifier.h"
#include "aos/common/sensors/sensor_packer.h"
#include "WPILib/Task.h"
#include "WPILib/Encoder.h"
#include "WPILib/DigitalInput.h"
#include "WPILib/Counter.h"

#include "frc971/queues/sensor_values.h"

namespace frc971 {

class SensorPacker
    : public ::aos::sensors::SensorPackerInterface<sensor_values> {
 public:
  SensorPacker();

  virtual void PackInto(sensor_values *values);

 private:
  Encoder lencoder;
  Encoder rencoder;
};

}  // namespace frc971

#endif  // FRC971_INPUT_SENSOR_PACKER_H_
