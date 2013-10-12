#ifndef GYRO_BOARD_FILL_PACKET_H_
#define GYRO_BOARD_FILL_PACKET_H_

#include <stdint.h>

#define DATA_STRUCT_NAME DataStruct
#include "data_struct.h"
#undef DATA_STRUCT_NAME

// Gets called in the USB data output ISR. Assumes that it will not be preempted
// except by very high priority things.
//
// Implemented in encoder.c because it depends on so many things in there.
void fillSensorPacket(struct DataStruct *packet);

#endif  // GYRO_BOARD_FILL_PACKET_H_
