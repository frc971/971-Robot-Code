#include "motors/core/kinetis.h"

#include <stdio.h>

#include "motors/core/time.h"
#include "motors/usb/usb_serial.h"
#include "motors/util.h"

namespace frc971 {
namespace salsa {

extern "C" {
void *__stack_chk_guard = (void *)0x67111971;
extern void usb_init();
int _write(int file, char *ptr, int len) {
  (void)file;
  return usb_serial_write(0, ptr, len);
}

void __stack_chk_fail(void);

extern char *__brkval;
extern uint32_t __bss_ram_start__[];
extern uint32_t __heap_start__[];
extern uint32_t __stack_end__[];

}  // extern "C"

extern "C" int main(void) {
  // for background about this startup delay, please see these conversations
  // https://forum.pjrc.com/threads/36606-startup-time-(400ms)?p=113980&viewfull=1#post113980
  // https://forum.pjrc.com/threads/31290-Teensey-3-2-Teensey-Loader-1-24-Issues?p=87273&viewfull=1#post87273
  delay(400);

  // Set all interrupts to the second-lowest priority to start with.
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_SANE_PRIORITY(i, 0xD);

  // Now set priorities for all the ones we care about. They only have meaning
  // relative to each other, which means centralizing them here makes it a lot
  // more manageable.
  NVIC_SET_SANE_PRIORITY(IRQ_USBOTG, 0x7);

  // Set the LED's pin to output mode.
  GPIO_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  usb_serial_init();
  usb_descriptor_set_product_id(0x0490);
  usb_init();

  // Give everything a chance to get going.
  delay(100);

  printf("Ram start:   %p\n", __bss_ram_start__);
  printf("Heap start:  %p\n", __heap_start__);
  printf("Heap end:    %p\n", __brkval);
  printf("Stack start: %p\n", __stack_end__);

  GPIOC_PSOR = 1 << 5;
  while (true) {}

  return 0;
}

void __stack_chk_fail(void) {
  while (true) {
    GPIOC_PSOR = (1 << 5);
    printf("Stack corruption detected\n");
    delay(1000);
    GPIOC_PCOR = (1 << 5);
    delay(1000);
  }
}

}  // namespace salsa
}  // namespace frc971
