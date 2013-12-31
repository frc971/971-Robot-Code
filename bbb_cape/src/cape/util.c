#include "cape/util.h"

#include "cape/led.h"

void led_write(uint32_t value, int bits) {
  for (int i = -2; i < bits; ++i) {
    led_set(LED_Z, i < 0);
    for (int ii = 0; ii < 1000000; ++ii) {
      led_set(LED_ERR, i >= 0 && ii < 500000);
      if (i >= 0) {
        led_set(LED_DB, value & (1 << i));
      } else {
        led_set(LED_DB, 0);
      }
    }
  }
}
