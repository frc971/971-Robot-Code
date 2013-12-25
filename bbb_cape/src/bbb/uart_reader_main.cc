#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"
#include "bbb/gpios.h"
#include "bbb/uart_reader.h"

using ::aos::time::Time;

#define DO_RESET 0

int main() {
  ::aos::Init();

#if DO_RESET
  // time since last good packet before we reset 
  // the board.
  static const Time kPacketTimeout = Time::InSeconds(1);

  ::bbb::Pin reset_pin = bbb::Pin(1, 6);
  reset_pin.MakeOutput();
#endif

  ::bbb::UartReader receiver(1500000);
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

  Time last_packet_time = Time::Now();
  while (true) {
#if DO_RESET
    if (!last_packet_time.IsWithin(Time::Now(), kPacketTimeout.ToNSec())) {
      LOG(ERROR, "No good packets for too long. Resetting cape.\n");
      reset_pin.Write(1);
      ::aos::time::SleepFor(Time::InSeconds(1));
      reset_pin.Write(0);
      
      last_packet_time = Time::Now();
    }
#endif

    if (!receiver.ReadPacket()) {
      LOG(WARNING, "Could not read a packet.\n");
      continue;
    }
    last_packet_time = Time::Now();

    const DataStruct *packet = receiver.get_packet<DataStruct>();
    LOG(DEBUG, "got one!\n");
    LOG(DEBUG, "timestamp %" PRIu64 "\n", packet->timestamp);
    for (int i = 0; i < 8; ++i) {
      LOG(DEBUG, "enc[%d]=%" PRId32 "\n", i, packet->main.encoders[i]);
    }
  }

  ::aos::Cleanup();
}
