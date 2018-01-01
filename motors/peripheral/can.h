#ifndef PERIPHERAL_CAN_H_
#define PERIPHERAL_CAN_H_

#include <stdint.h>

// The code defined here calls functions in vesc/vesc_can.h from various
// interrupts and expects them to call back into this file to do something.

#ifdef __cplusplus
extern "C" {
#endif

void can_init(void);

int can_send(uint32_t can_id, const unsigned char *data, unsigned int length);

// Sets *length to -1 if there isn't a new piece of data to receive.
void can_receive_command(unsigned char *data, int *length);

#ifdef __cplusplus
}
#endif

#endif  // PERIPHERAL_CAN_H_
