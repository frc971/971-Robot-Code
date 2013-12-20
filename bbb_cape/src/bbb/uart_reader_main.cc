#include <stdint.h>

#include "aos/atom_code/init.h"
#include "aos/common/logging/logging_impl.h"
#include "aos/common/time.h"
#include "bbb/gpios.h"
#include "bbb/uart_reader.h"

using ::aos::time::Time;

int main() {
  aos::Init();

  // time since last good packet before we reset 
  // the board.
  Time kPacketTimeout = Time::InSeconds(1);

  bbb::UartReader receiver(3000000);
  bbb::Pin reset_pin = bbb::Pin(1, 6);
  reset_pin.MakeOutput();

  DataStruct packet;
  Time last_packet_time = Time::Now();
  while (true) {
    if (!last_packet_time.IsWithin(Time::Now(),
      kPacketTimeout.ToNSec())) {
      
      LOG(ERROR, "No good packets for too long. Resetting cape.\n");
      reset_pin.Write(1);
      ::aos::time::SleepFor(Time::InSeconds(1));
      reset_pin.Write(0);
      
      last_packet_time = Time::Now();
    }

    if (receiver.GetPacket(&packet)) {
      LOG(INFO, "Could not get a packet.\n");
      continue;
    }
    last_packet_time = Time::Now();

    //TODO (danielp): Do stuff here with the data we got.
  }

  aos::Cleanup();
  return 0;
}
