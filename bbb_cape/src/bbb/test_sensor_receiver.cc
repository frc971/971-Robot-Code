#include <inttypes.h>

#include "aos/linux_code/init.h"

#include "bbb/sensor_reader.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

double gyro_translate(int64_t in) {
  return in / 16.0 / 1000.0 / (180.0 / M_PI);
}

void PacketReceived(const ::bbb::DataStruct *data,
                    const ::aos::time::Time &cape_timestamp) {
  LOG(DEBUG, "cape timestamp %010" PRId32 ".%09" PRId32 "s\n",
      cape_timestamp.sec(), cape_timestamp.nsec());
  if (data->uninitialized_gyro) {
    LOG(DEBUG, "uninitialized gyro\n");
  } else if (data->zeroing_gyro) {
    LOG(DEBUG, "zeroing gyro\n");
  } else if (data->bad_gyro) {
    LOG(ERROR, "bad gyro\n");
  } else if (data->old_gyro_reading) {
    LOG(WARNING, "old/bad gyro reading\n");
  } else {
    LOG(DEBUG, "gyro reading is %f\n", gyro_translate(data->gyro_angle));
  }

  LOG(DEBUG, "digitals=%x\n", data->test.digitals);
  for (int i = 0; i < 8; ++i) {
    LOG(DEBUG, "enc[%d]=%" PRId32 "\n", i, data->test.encoders[i]);
    LOG(DEBUG, "adc[%d]=%f (%" PRIx16 ")\n", i,
        3.3 * data->test.analogs[i] / 0x3FF, data->test.analogs[i]);
  }
}

}  // namespace

int main() {
  ::aos::Init(::bbb::SensorReader::kRelativePriority);
  ::bbb::SensorReader reader("test");
  while (true) {
    PacketReceived(reader.ReadPacket(), reader.GetCapeTimestamp());
  }
  ::aos::Cleanup();
}
