#ifndef CAPE_ROBOT_H_
#define CAPE_ROBOT_H_

// This header file is for the robot-specific files which should be named
// robot_name.c. It documents what functions each robot should implement.

#include "cape/fill_packet.h"

// Has a default (weak empty) definition.
void robot_init(void);

void robot_fill_packet(struct DataStruct *packet);

#endif  // CAPE_ROBOT_H_
