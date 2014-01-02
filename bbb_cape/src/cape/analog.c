#include "cape/analog.h"

#include <string.h>

#include <STM32F2XX.h>

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

uint16_t analog_readings[NUM_CHANNELS] __attribute__((aligned(8)));
static volatile int current_channel;
static volatile int partial_reading;
static volatile int frame;

static void start_read(int channel) {
  // This needs to wait 13 cycles between enabling the CSEL pin and starting to
  // send data.
  // (100ns+8ns)*120MHz = 12.96

  // Clear the CSEL pin to select it.
  for (int i = 0; i < 9; ++i) gpio_off(CSEL_GPIO, CSEL_NUM);
  current_channel = channel;
  partial_reading = 0;
  frame = 0;
  SPI->DR = 1;  // start bit
  uint16_t data = (1 << 15) /* not differential */ |
      (channel << 12);
  while (!(SPI->SR & SPI_SR_TXE));
  SPI->DR = data;
}

void SPI_IRQHandler(void) {
  uint32_t status = SPI->SR;
  if (status & SPI_SR_RXNE) {
    uint16_t value = SPI->DR;
    if (frame == 0) {
      frame = 1;
      partial_reading = value;
    } else {
      // Masking off the high bits is important because there's nothing driving
      // the MISO line during the time the MCU receives them.
      analog_readings[current_channel] = (partial_reading << 16 | value) & 0x3FF;
      for (int i = 0; i < 100; ++i) gpio_off(CSEL_GPIO, CSEL_NUM);
      gpio_on(CSEL_GPIO, CSEL_NUM);

      TIM->CR1 = TIM_CR1_OPM;
      TIM->EGR = TIM_EGR_UG;
      TIM->CR1 |= TIM_CR1_CEN;
    }
  }
}

void TIM_IRQHandler(void) {
  TIM->SR = ~TIM_SR_CC1IF;

  start_read((current_channel + 1) % NUM_CHANNELS);
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

  TIM->CR1 = TIM_CR1_OPM;
  TIM->DIER = TIM_DIER_CC1IE;
  TIM->CCMR1 = 0;
  // Make each tick take 1500ns.
  TIM->PSC = (60 * 1500 / 1000) - 1;
  // Call the interrupt after 1 tick.
  TIM->CCR1 = 1;

  SPI->CR1 = 0;  // make sure it's disabled
  SPI->CR1 =
      SPI_CR1_DFF /* 16 bit frame */ |
      SPI_CR1_SSM | SPI_CR1_SSI | /* don't watch for other masters */
      3 << 3 /* 30MHz/16 = 1.875MHz */ |
      SPI_CR1_MSTR /* master mode */;
  SPI->CR2 = SPI_CR2_RXNEIE;
  SPI->CR1 |= SPI_CR1_SPE;  // enable it

  start_read(0);
}
