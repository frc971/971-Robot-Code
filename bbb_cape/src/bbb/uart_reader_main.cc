#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging_impl.h"

#include "bbb/uart_reader.h"
#include "bbb/packet_finder.h"
#include "bbb/data_struct.h"

int main() {
  ::aos::Init();

  ::bbb::UartReader reader(750000);
  ::bbb::PacketFinder receiver(&reader, DATA_STRUCT_SEND_SIZE - 4);
  receiver.ReadPacket();
  // TODO(brians): Do this cleanly.
  int chrt_result = system(
      "bash -c 'chrt -r -p 55 $(top -n1 | fgrep irq/89 | cut -d\" \" -f2)'");
  if (chrt_result == -1) {
    LOG(FATAL, "system(chrt -r -p 55 the_irq) failed\n");
  } else if (!WIFEXITED(chrt_result) || WEXITSTATUS(chrt_result) != 0) {
    LOG(FATAL, "$(chrt -r -p 55 the_irq) failed, return value = %d\n",
        WEXITSTATUS(chrt_result));
  }

  while (true) {
    if (!receiver.ReadPacket()) {
      LOG(WARNING, "Could not read a packet.\n");
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
