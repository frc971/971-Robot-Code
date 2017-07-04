#include "motors/core/kinetis.h"

#include <stdio.h>

#include <atomic>

#include "motors/core/time.h"
#include "motors/motor.h"
#include "motors/motor_controls.h"
#include "motors/peripheral/adc.h"
#include "motors/peripheral/can.h"
#include "motors/usb/usb_serial.h"
#include "motors/util.h"

namespace frc971 {
namespace salsa {
namespace {

::std::atomic<Motor *> global_motor{nullptr};

extern "C" {

void *__stack_chk_guard = (void *)0x67111971;
void __stack_chk_fail(void) {
  while (true) {
    GPIOC_PSOR = (1 << 5);
    printf("Stack corruption detected\n");
    delay(1000);
    GPIOC_PCOR = (1 << 5);
    delay(1000);
  }
}

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

void ftm0_isr(void) {
  global_motor.load(::std::memory_order_relaxed)->HandleInterrupt();
}

}  // extern "C"
}  // namespace

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
  NVIC_SET_SANE_PRIORITY(IRQ_FTM0, 0x3);

  // Set the LED's pin to output mode.
  GPIO_BITBAND(GPIOC_PDDR, 5) = 1;
  PORTC_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  GPIO_BITBAND(GPIOA_PDDR, 15) = 1;
  PORTA_PCR15 = PORT_PCR_DSE | PORT_PCR_MUX(1);

  DMA_CR = DMA_CR_EMLM;
  usb_serial_init();
  usb_descriptor_set_product_id(0x0490);
  usb_init();
  AdcInit();
  MathInit();
  delay(1000);
  can_init();

  MotorControlsImplementation controls;

  delay(1000);
  Motor motor(FTM0, FTM1, &controls);
  motor.Init();
  global_motor.store(&motor, ::std::memory_order_relaxed);
  // Output triggers to things like the PDBs on initialization.
  FTM0_EXTTRIG = FTM_EXTTRIG_INITTRIGEN;
  // Don't let any memory accesses sneak past here, because we actually
  // need everything to be starting up.
  __asm__("" :: : "memory");

  // Give everything a chance to get going.
  delay(100);

#if 0
  printf("Ram start:   %p\n", __bss_ram_start__);
  printf("Heap start:  %p\n", __heap_start__);
  printf("Heap end:    %p\n", __brkval);
  printf("Stack start: %p\n", __stack_end__);
#endif

  printf("Going silent to zero motors...\n");
  // Give the print a chance to make it out.
  delay(1000);
  motor.Zero();

  printf("Zeroed motor!\n");
  // Give stuff a chance to recover from interrupts-disabled.
  delay(100);
  motor.Start();

  GPIOA_PCOR = 1 << 15;

  // TODO(Brian): Use SLEEPONEXIT to reduce interrupt latency?
  while (true) {}

  return 0;
}

}  // namespace salsa
}  // namespace frc971
