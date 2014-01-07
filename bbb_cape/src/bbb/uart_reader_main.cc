#include <stdint.h>
#include <inttypes.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"
#include "bbb/gpo.h"
#include "bbb/uart_reader.h"
#include "bbb/packet_finder.h"
#include "bbb/data_struct.h"

int main() {
  ::aos::Init();

  ::bbb::Gpo reset_pin(1, 6);

  ::bbb::UartReader reader(750000);
  ::bbb::PacketFinder receiver(&reader, DATA_STRUCT_SEND_SIZE - 4);

  while (true) {
    if (!receiver.ReadPacket(::aos::time::Time::Now() +
                             ::aos::time::Time::InSeconds(1.5))) {
      LOG(WARNING, "Could not read a packet. (Resetting board...)\n");
      reset_pin.Set(true);
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(1));
      reset_pin.Set(false);
      continue;
    }

    const ::bbb::DataStruct *packet = receiver.get_packet< ::bbb::DataStruct>();
    LOG(DEBUG, "got one!\n");
    LOG(DEBUG, "timestamp %" PRIu64 "\n", packet->timestamp);
    LOG(DEBUG, "gyro old=%d uninit=%d z=%d bad=%d %" PRId64 " \n",
        packet->old_gyro_reading, packet->uninitialized_gyro,
        packet->zeroing_gyro, packet->bad_gyro, packet->gyro_angle);
    for (int i = 0; i < 8; ++i) {
      LOG(DEBUG, "enc[%d]=%" PRId32 "\n", i, packet->test.encoders[i]);
      LOG(DEBUG, "adc[%d]=%f (%" PRIx16 ")\n", i,
          3.3 * packet->test.analogs[i] / 0x3FF, packet->test.analogs[i]);
    }
    LOG(DEBUG, "digitals=%x\n", packet->test.digitals);
    LOG(DEBUG, "+=%" PRId32 "/%" PRIu8 " and -=%" PRId32 "/%" PRIu8 "\n",
        packet->test.posedge_value, packet->test.posedge_count,
        packet->test.negedge_value, packet->test.negedge_count);
  }

  ::aos::Cleanup();
}
