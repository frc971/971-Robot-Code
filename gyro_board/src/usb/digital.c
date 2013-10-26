#include "digital.h"

inline int readGPIO_inline(int major, int minor) {
  switch (major) {
    case 0:
      return readGPIO(GPIO0, minor);
    case 1:
      return readGPIO(GPIO1, minor);
    case 2:
      return readGPIO(GPIO2, minor);
    default:
      return -1;
  }
}

int dip_switch(int channel) {
  switch (channel) {
    case 0:
      return readGPIO(GPIO1, 29);
    case 1:
      return readGPIO(GPIO2, 13);
    case 2:
      return readGPIO(GPIO0, 11);
    case 3:
      return readGPIO(GPIO0, 10);
    default:
      return -1;
  }
}

int is_bot3;
void digital_init(void) {
  is_bot3 = 0;
}
