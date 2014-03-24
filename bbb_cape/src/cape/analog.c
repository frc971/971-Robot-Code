#include "cape/analog.h"

#include <string.h>

#include "cape/util.h"
#include "cape/led.h"

#define SPI SPI2
#define SPI_IRQHandler SPI2_IRQHandler
#define SPI_IRQn SPI2_IRQn
#define RCC_APB1ENR_SPIEN RCC_APB1ENR_SPI2EN
#define TIM TIM14
#define TIM_IRQHandler TIM8_TRG_COM_TIM14_IRQHandler
#define TIM_IRQn TIM8_TRG_COM_TIM14_IRQn
#define RCC_APB1ENR_TIMEN RCC_APB1ENR_TIM14EN
#define CSEL_GPIO GPIOB
#define CSEL_NUM 12

#define NUM_CHANNELS 8

// This file handles reading values from the MCP3008-I/SL ADC.

uint16_t analog_readings[NUM_CHANNELS] __attribute__((aligned(8)));
static volatile int current_channel;
static volatile int partial_reading;
static volatile int frame;
static volatile int analog_errors;

void SPI_IRQHandler(void) {
  uint32_t status = SPI->SR;
  if (status & SPI_SR_RXNE) {
    uint16_t value = SPI->DR;
    if (frame == 0) {
      frame = 1;
      partial_reading = value;
    } else {
      frame = 2;
      // Masking off the high bits is important because there's nothing driving
      // the MISO line during the time the MCU receives them.
      analog_readings[current_channel] =
          (partial_reading << 16 | value) & 0x3FF;
      for (int i = 0; i < 100; ++i) gpio_off(CSEL_GPIO, CSEL_NUM);
      gpio_on(CSEL_GPIO, CSEL_NUM);

      current_channel = (current_channel + 1) % NUM_CHANNELS;
    }
  }
}

void TIM_IRQHandler(void) {
  TIM->SR = ~TIM_SR_UIF;

  if (frame != 2) {
    // We're not done with the previous reading yet, so we're going to reset and
    // try again.
    // 270ns*120MHz = 32.4
    for (int i = 0; i < 33; ++i) gpio_on(CSEL_GPIO, CSEL_NUM);
    ++analog_errors;
  }

  // This needs to wait 13 cycles between enabling the CSEL pin and starting to
  // send data.
  // (100ns+8ns)*120MHz = 12.96

  // Clear the CSEL pin to select it.
  for (int i = 0; i < 9; ++i) gpio_off(CSEL_GPIO, CSEL_NUM);
  partial_reading = 0;
  frame = 0;
  SPI->DR = 1;  // start bit
  uint16_t data = (1 << 15) /* not differential */ |
      (current_channel << 12);
  while (!(SPI->SR & SPI_SR_TXE));
  SPI->DR = data;
}

void analog_init(void) {
  memset(analog_readings, 0xFF, sizeof(analog_readings));

  RCC->APB1ENR |= RCC_APB1ENR_SPIEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIMEN;

  gpio_setup_out(CSEL_GPIO, CSEL_NUM, 3);
  gpio_on(CSEL_GPIO, CSEL_NUM);  // deselect it

  gpio_setup_alt(GPIOB, 13, 5);  // SCK
  gpio_setup_alt(GPIOB, 14, 5);  // MISO
  gpio_setup_alt(GPIOB, 15, 5);  // MOSI

  NVIC_SetPriority(SPI_IRQn, 6);
  NVIC_EnableIRQ(SPI_IRQn);
  NVIC_SetPriority(TIM_IRQn, 6);
  NVIC_EnableIRQ(TIM_IRQn);

  // We set it up to trigger at 4.44KHz (each sensor at just over 500Hz).
  // 1/(1.875MHz)*32 = 17067ns (58.6Khz), and we don't want to be close (other
  // interrupts do get in the way, and there's no reason to be).
  TIM->CR1 = 0;
  TIM->DIER = TIM_DIER_UIE;
  // Make each tick take 45000ns.
  TIM->PSC = (60 * 45000 / 1000) - 1;
  // Only count to 5 before triggering the interrupt and wrapping around.
  TIM->ARR = 5;

  SPI->CR1 = 0;  // make sure it's disabled
  SPI->CR1 =
      SPI_CR1_DFF /* 16 bit frame */ |
      SPI_CR1_SSM | SPI_CR1_SSI | /* don't watch for other masters */
      3 << 3 /* 30MHz/16 = 1.875MHz */ |
      SPI_CR1_MSTR /* master mode */;
  SPI->CR2 = SPI_CR2_RXNEIE;
  SPI->CR1 |= SPI_CR1_SPE;  // enable it

  current_channel = 0;
  analog_errors = 0;

  TIM->EGR = TIM_EGR_UG;
  TIM->CR1 |= TIM_CR1_CEN;
}

int analog_get_errors(void) {
  NVIC_DisableIRQ(TIM_IRQn);
  int r = analog_errors;
  analog_errors = 0;
  NVIC_EnableIRQ(TIM_IRQn);
  return r;
}
