#include "cape/robot.h"

#include "cape/encoder.h"
#include "cape/analog.h"
#include "cape/digital.h"

#define CAPTURE_NUM 11

#define CAPTURE(num, np) digital_capture_ ## num ## np
#define CAPTURE2(num, np) CAPTURE(num, np)
#define CAPTURE_P CAPTURE2(CAPTURE_NUM, P)
#define CAPTURE_N CAPTURE2(CAPTURE_NUM, N)

static int32_t posedge_value, negedge_value;
static uint8_t posedge_count = 0, negedge_count = 0;

void CAPTURE_P(void) {
  ++posedge_count;
  posedge_value = encoder_read(1);
}
void CAPTURE_N(void) {
  ++negedge_count;
  negedge_value = encoder_read(1);
}

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

  packet->test.digitals = 0;
  for (int i = 0; i < 12; ++i) {
    SET_BITS(packet->test.digitals, 1, digital_read(i), i);
  }

  digital_capture_disable(CAPTURE_NUM);
  packet->test.posedge_count = posedge_count;
  packet->test.posedge_value = posedge_value;
  packet->test.negedge_count = negedge_count;
  packet->test.negedge_value = negedge_value;
  digital_capture_enable(CAPTURE_NUM);
}
