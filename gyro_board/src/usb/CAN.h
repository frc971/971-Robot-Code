#ifndef _CAN_H_
#define _CAN_H_

typedef struct _can_message {
  unsigned RTR:1;
  unsigned error:7;
  uint16_t id;
  uint8_t priority;
  uint8_t length;
  union {
    uint8_t data[8];
    uint32_t wdata[2];
  };
} can_message;

int CAN_get(can_message *message);

// Sets up PCLKSEL for CAN stuff (errata PCLKSELx.1).
void CAN_PCLKSEL(void);
void initCAN(void);

#endif
