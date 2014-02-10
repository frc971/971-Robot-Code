#include <stdint.h>

#include <STM32F2XX.h>

#include "cape/bootloader_handoff.h"
#include "cape/led.h"
#include "cape/util.h"

// Actually runs the bootloader code.
// Implemented in bootloader_impl.c.
void bootloader_start(void) __attribute__((noreturn));

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

static void setup_main_clock(void) {
  // We set up a couple of registers here separately from the actual register to
  // avoid having to think about what happens when the value is in some
  // intermediate state.

  RCC->CR |= RCC_CR_HSEON;  // get the HSE oscillator going

  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN |
               FLASH_ACR_LATENCY_3WS;

  uint32_t rcc_pllcfgr = 0;
  rcc_pllcfgr |= RCC_PLLCFGR_PLLSRC;  // use the external oscillator
  // 8MHz * 120 / 4 / 2 = 120MHz
  rcc_pllcfgr |= 120 << 6;  // multiplier
  rcc_pllcfgr |= 4;  // divider
  rcc_pllcfgr |= 5 << 24;  // other stuff divider = 5, just in case
  RCC->PLLCFGR = rcc_pllcfgr;

  uint32_t rcc_cfgr = 0;
  rcc_cfgr |= 4 << 13;  // APB2 divider = 2
  rcc_cfgr |= 5 << 10;  // APB1 divider = 4
  rcc_cfgr |= 2;  // use the PLL

  // Wait for the HSE oscillator to be stable.
  while (!(RCC->CR & RCC_CR_HSERDY)) {}

  RCC->CR |= RCC_CR_PLLON;
  // Wait for the main PLL to be stable.
  while (!(RCC->CR & RCC_CR_PLLRDY)) {}
  // Wait until the flash is using 3 wait states.
  while ((FLASH->ACR & 7) != 3) {}
  RCC->CFGR = rcc_cfgr;
  // Wait until we are using the PLL as our main clock source.
  while ((RCC->CFGR & (3 << 2)) != (2 << 2)) {}
}

void _start(void) {
  // Enable the GPIO pin clocks.
  // We don't have anything attached to the 1 port D pin, so don't bother
  // enabling it.
  RCC->AHB1ENR |=
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
  led_init();
  led_set(LED_HB, 1);
  gpio_set_pupd(GPIOC, 2, 2);

  setup_main_clock();

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;  // enable IO compensation cell
  while (!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY)) {}  // wait for it to be ready

  if (GPIOC->IDR & (1 << 2)) {
    jump_to_main();
  } else {
    bootloader_start();
  }
}
