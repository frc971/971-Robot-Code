#include <stdint.h>

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
  ::bbb::UartReader receiver(300000);
  //::bbb::UartReader receiver(19200);

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

    DataStruct packet;
    if (!receiver.GetPacket(&packet)) {
      LOG(WARNING, "Could not get a packet.\n");
      continue;
    }
    last_packet_time = Time::Now();

    LOG(DEBUG, "got one!\n");
    //TODO (danielp): Do stuff here with the data we got.
  }

  ::aos::Cleanup();
}
