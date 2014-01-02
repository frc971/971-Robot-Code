#ifndef CAPE_FILL_PACKET_H_
#define CAPE_FILL_PACKET_H_

#include <stdint.h>

#include "cape/util.h"
#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

// Starts writing out sensor packets as fast as the serial port can write them.
void fill_packet_start(void);

#endif  // CAPE_FILL_PACKET_H_
