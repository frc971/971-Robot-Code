#ifndef PERIPHERAL_CAN_H_
#define PERIPHERAL_CAN_H_

#include <stdint.h>

// The code defined here calls functions in vesc/vesc_can.h from various
// interrupts and expects them to call back into this file to do something.

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_EFF_FLAG UINT32_C(0x80000000) /* EFF/SFF is set in the MSB */

void can_init(uint32_t id0, uint32_t id1);

// Mailbox is 2-7 (inclusive) for the send mailboxes.
int can_send(uint32_t can_id, const unsigned char *data, unsigned int length,
             unsigned int mailbox);

// Sets *length to -1 if there isn't a new piece of data to receive.
// Mailbox is 0 or 1 for the two receive mailboxes.
void can_receive(unsigned char *data, int *length, int mailbox);

#ifdef __cplusplus
}
#endif

#endif  // PERIPHERAL_CAN_H_
