#include "cape/robot.h"

#include "cape/encoder.h"
#include "cape/analog.h"
#include "cape/digital.h"

void robot_fill_packet(struct DataStruct *packet) {
  packet->main.encoders[0] = encoder_read(0);
  packet->main.encoders[1] = encoder_read(1);
  packet->main.encoders[2] = encoder_read(2);
  packet->main.encoders[3] = encoder_read(3);
  packet->main.encoders[4] = encoder_read(4);
  packet->main.encoders[5] = encoder_read(5);
  packet->main.encoders[6] = encoder_read(6);
  packet->main.encoders[7] = encoder_read(7);

  for (int i = 0; i < 8; ++i) {
    packet->main.analogs[i] = analog_get(i);
  }

  // TODO(brians): digitals
}
