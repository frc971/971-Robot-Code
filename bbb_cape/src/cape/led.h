#ifndef CAPE_LED_H_
#define CAPE_LED_H_

// The LEDs as referenced by the silkscreen.
enum LED {
  LED_ERR,
  LED_HB,
  LED_Z,
  LED_DB,
};

// Turns the indicated LED on or off.
void led_set(enum LED led, int on);

void led_init(void);

#endif  // CAPE_LED_H_
