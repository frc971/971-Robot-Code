#include "cape/robot.h"

#include "cape/encoder.h"
#include "cape/analog.h"
#include "cape/digital.h"

void robot_fill_packet(struct DataStruct *packet) {
  packet->test.encoders[0] = encoder_read(0);
  packet->test.encoders[1] = encoder_read(1);
  packet->test.encoders[2] = encoder_read(2);
  packet->test.encoders[3] = encoder_read(3);
  packet->test.encoders[4] = encoder_read(4);
  packet->test.encoders[5] = encoder_read(5);
  packet->test.encoders[6] = encoder_read(6);
  packet->test.encoders[7] = encoder_read(7);

  for (int i = 0; i < 8; ++i) {
    packet->test.analogs[i] = analog_get(i);
  }

  // TODO(brians): digitals
}
