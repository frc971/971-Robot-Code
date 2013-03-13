#include "frc971/input/sensor_packer.h"

#include <arpa/inet.h>

#include "aos/common/inttypes.h"

using ::aos::MutexLocker;

namespace frc971 {

SensorPacker::SensorPacker() : lencoder(1, 2), rencoder(3, 4) {
  lencoder.Start();
  rencoder.Start();

  printf("frc971::SensorPacker started\n");
}

void SensorPacker::PackInto(sensor_values *values) {
  values->lencoder = htonl(-lencoder.GetRaw());
  values->rencoder = -htonl(-rencoder.GetRaw());
}

}  // namespace frc971
