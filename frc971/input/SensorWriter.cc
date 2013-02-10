#include <arpa/inet.h>

#include "WPILib/Task.h"
#include "WPILib/Encoder.h"
#include "WPILib/DigitalInput.h"
#include "WPILib/Counter.h"

#include "aos/aos_core.h"
#include "aos/crio/motor_server/SensorOutput.h"
#include "aos/common/inttypes.h"
#include "aos/common/mutex.h"
#include "aos/crio/shared_libs/interrupt_notifier.h"

#include "frc971/queues/sensor_values.h"

using ::aos::MutexLocker;

namespace frc971 {

class SensorWriter : public aos::SensorOutput<sensor_values> {
  Encoder lencoder;
  Encoder rencoder;

 public:
  SensorWriter() : lencoder(1, 2), rencoder(3, 4) {
    lencoder.Start();
    rencoder.Start();

    printf("frc971::SensorWriter started\n");
  }

  virtual void RunIteration(sensor_values &vals) {
    vals.lencoder = htonl(-lencoder.GetRaw());
    vals.rencoder = -htonl(-rencoder.GetRaw());
  }
};

}  // namespace frc971

AOS_RUN(frc971::SensorWriter)
