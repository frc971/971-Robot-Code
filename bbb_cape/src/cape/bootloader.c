#include <stdint.h>

#include <STM32F2XX.h>

#include "cape/bootloader_handoff.h"

// Sets everything up and then jumps to the main code.
static void jump_to_main(void) __attribute__((noreturn));
static void jump_to_main(void) {
  __asm__ __volatile__(
      "mov sp, %[stack]\n\t"
      "bx %[reset]" : :
      [stack]"r"(RAM_START + RAM_SIZE), [reset]"r"(MAIN_FLASH_START | 1)
      : "memory");
  __builtin_unreachable();
}

void _start(void) {
  SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;  // enable IO compensation cell
  while (!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY)) {}  // wait for it to be ready

  // We don't have anything on the 1 port D pin, so don't bother enabling it.
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  jump_to_main();
}
