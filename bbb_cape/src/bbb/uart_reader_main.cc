#include <stdint.h>
#include <inttypes.h>

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

  //::bbb::UartReader receiver(3000000);
  ::bbb::UartReader receiver(30000);

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
    LOG(DEBUG, "0=%d\n", packet->main.encoders[0]);
    //TODO (danielp): Do stuff here with the data we got.
  }

  ::aos::Cleanup();
}
