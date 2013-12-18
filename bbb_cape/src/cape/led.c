#include "cape/led.h"

#include <STM32F2XX.h>

#include "cape/util.h"

#define LED_SPEED 0

// DB = PC3
// Z = PB1
// HB = PB4
// ERR = PB11

static void do_led_set(GPIO_TypeDef *port, int number, int on) {
  if (on) {
    port->BSRRH = 1 << number;
  } else {
    port->BSRRL = 1 << number;
  }
}

void led_set(enum LED led, int on) {
  switch (led) {
    case LED_ERR:
      do_led_set(GPIOB, 11, on);
      break;
    case LED_HB:
      do_led_set(GPIOB, 4, on);
      break;
    case LED_Z:
      do_led_set(GPIOB, 1, on);
      break;
    case LED_DB:
      do_led_set(GPIOC, 3, on);
      break;
  }
}

void led_init(void) {
  gpio_setup_out(GPIOB, 11, LED_SPEED);
  led_set(LED_ERR, 0);
  gpio_setup_out(GPIOB, 4, LED_SPEED);
  led_set(LED_HB, 0);
  gpio_setup_out(GPIOB, 1, LED_SPEED);
  led_set(LED_Z, 0);
  gpio_setup_out(GPIOC, 3, LED_SPEED);
  led_set(LED_DB, 0);
}
