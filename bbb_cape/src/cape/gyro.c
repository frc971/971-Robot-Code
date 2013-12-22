#include "cape/gyro.h"

#include <inttypes.h>

#include <STM32F2XX.h>

#include "cape/util.h"
#include "cape/led.h"

#define printf(...)

#define SPI SPI3
#define SPI_IRQHandler SPI3_IRQHandler
#define SPI_IRQn SPI3_IRQn
#define RCC_APB1ENR_SPIEN RCC_APB1ENR_SPI3EN
#define TIM TIM13
#define TIM_IRQHandler TIM8_UP_TIM13_IRQHandler
#define TIM_IRQn TIM8_UP_TIM13_IRQn
#define RCC_APB1ENR_TIMEN RCC_APB1ENR_TIM13EN
#define CSEL_GPIO GPIOA
#define CSEL_NUM 4
// The header file also contains references to TIM in gyro_read.

struct GyroOutput gyro_output;

// Set when a parity error is detected and cleared before starting a read.
static volatile int parity_error;
// Which byte we're currently waiting to read.
static volatile int receive_byte;
// The first byte that we receive (the most significant one).
static volatile uint16_t high_value;

// 1 if the latest result is potentially bad and 0 if it's good.
static volatile int bad_reading;
// 1 if the gyro is bad adn we're not going to get any more readings.
static volatile int bad_gyro;
// The new reading waiting for the next timer cycle to be outputted.
static volatile int16_t new_reading;

struct GyroOutput gyro_output;

// How many times per second to read the gyro value.
#define kGyroReadFrequency 200
// How many times per second to flash the LED.
// Must evenly divide kGyroReadFrequency.
#define kFlashFrequency 10

#define kStartupCycles (kGyroReadFrequency * 2)
#define kZeroingCycles (kGyroReadFrequency * 6)

// An accumulator for all of the values read while zeroing.
int32_t zero_bias = 0;

int startup_cycles_left = kStartupCycles;
int zeroing_cycles_left = kZeroingCycles;

// These are a pair that hold the offset calculated while zeroing.
// full_units_ is the base (in ticks) and remainder_ ranges between 0 and
// kZeroingCycles (like struct timespec). remainder_ is used to calculate which
// cycles to add an additional unit to the result.
int32_t full_units_offset = 0;
int32_t remainder_offset = 0;
// This keeps track of when to add 1 to the read value (using _offset).
int32_t remainder_sum = 0;

int32_t led_flash = 0;

enum State {
  STATE_SETUP0,
  STATE_SETUP1,
  STATE_SETUP2,
  STATE_SETUP3,
  STATE_READ,
};
static volatile enum State state;
static int setup_counter;

// Switches to new_state in time TIM milliseconds (aka it shows in the TIM ISR).
static void switch_state(enum State new_state, int time) {
  TIM->CR1 = 0;
  state = new_state;
  TIM->CCR1 = time;
  TIM->EGR = TIM_EGR_UG;
  TIM->CR1 |= TIM_CR1_CEN;
}

static void gyro_setup_failed(void) {
  printf("gyro setup failed. stopping\n");
  gyro_output.angle = 0;
  gyro_output.last_reading_bad = gyro_output.gyro_bad = 1;
  gyro_output.initialized = 1;
  gyro_output.zeroed = 0;
  led_set(LED_ERR, 1);
}

static void gyro_enable_csel(void) {
  // Clear the CSEL pin to select it.
  // Do it 8 times (9 cycles) to wait for the amount of time the gyro datasheet
  // says we need to.
  // (1/2/(7.5MHz)+8ns)*120MHz = 8.96
  for (int i = 0; i < 8; ++i) CSEL_GPIO->BSRRL = 1 << CSEL_NUM;
}

// Blocks until there is space to enqueue data.
static void spi_write(uint16_t data) {
  while (!(SPI->SR & SPI_SR_TXE)) {}
  SPI->DR = data;
}

static void do_gyro_read(uint32_t data) {
  parity_error = 0;
  receive_byte = 0;

  gyro_enable_csel();
  spi_write(data >> 16);
  if (__builtin_parity(data & ~1) == 0) data |= 1;
  spi_write(data);
}

// Returns all of the non-data bits in the "header" except the parity from
// value.
static uint8_t gyro_status(uint32_t value) {
  return (value >> 26) & ~4;
}

// Returns all of the error bits in the "footer" from value.
static uint8_t gyro_errors(uint32_t value) {
  return (value >> 1) & 0x7F;
}

static void process_reading(int16_t reading) {
  switch (state) {
    case STATE_SETUP0:
      if (parity_error) {
        switch_state(STATE_SETUP0, 100);
      } else {
        if (reading != 1) {
          printf("gyro unexpected initial response 0x%"PRIx32"\n", reading);
          // There's a chance that we're retrying because of a parity error
          // previously, so keep going.
        }
        // Wait for it to assert the fault conditions before reading them.
        switch_state(STATE_SETUP1, 50);
      }
      break;
    case STATE_SETUP1:
      if (parity_error) {
        switch_state(STATE_SETUP0, 100);
      } else {
        // Wait for it to clear the fault conditions before reading again.
        switch_state(STATE_SETUP2, 50);
      }
      break;
    case STATE_SETUP2:
      if (parity_error) {
        switch_state(STATE_SETUP0, 100);
      } else {
        // If it's not reporting self test data.
        if (gyro_status(reading) != 2) {
          printf("gyro first value 0x%"PRIx32" not self test data\n", reading);
          switch_state(STATE_SETUP0, 100);
          break;
        }
        // If we don't see all of the errors.
        if (gyro_errors(reading) != 0x7F) {
          printf("gyro self test value 0x%"PRIx32" is bad\n", reading);
          gyro_setup_failed();
          break;
        }
        // Wait for the sequential transfer delay before reading out the last of
        // the self test data.
        switch_state(STATE_SETUP3, 1);
      }
      break;
    case STATE_SETUP3:
      if (parity_error) {
        switch_state(STATE_SETUP0, 100);
      } else {
        // It should still be reporting self test data.
        if (gyro_status(reading) != 2) {
          printf("gyro second value 0x%"PRIx32" not self test data\n", reading);
          switch_state(STATE_SETUP0, 100);
          break;
        }

        gyro_output.initialized = 1;
        gyro_output.angle = 0;
        gyro_output.last_reading_bad = 1;  // until we're started up
        gyro_output.gyro_bad = 0;
        // Start reading values (after the sequential transfer delay).
        switch_state(STATE_READ, 1);
      }
      break;
    case STATE_READ:
      new_reading = reading;
      switch_state(STATE_READ, 1000 / kGyroReadFrequency);
      break;
  }
}

static void reading_received(uint32_t value) {
  if (parity_error) {
    bad_reading = 1;
  } else {
    // This check assumes that the sequence bits are all 0, but they should be
    // because that's all we send.
    if (gyro_status(value) != 1) {
      uint8_t status = gyro_status(value);
      if (status == 0) {
        printf("gyro says sensor data is bad\n");
      } else {
        printf("gyro gave weird status 0x%"PRIx8"\n", status);
      }
      bad_reading = 1;
    }

    if (gyro_errors(value) != 0) {
      uint8_t errors = gyro_errors(value);
      if (errors & ~(1 << 1)) {
        bad_reading = 1;
        // Error 1 (continuous self-test error) will set status to 0 if it's bad
        // enough by itself.
      }
      if (errors & (1 << 6)) {
        printf("gyro PLL error\n");
      }
      if (errors & (1 << 5)) {
        printf("gyro quadrature error\n");
      }
      if (errors & (1 << 4)) {
        printf("gyro non-volatile memory error\n");
        bad_gyro = 1;
      }
      if (errors & (1 << 3)) {
        printf("gyro volatile memory error\n");
        bad_gyro = 1;
      }
      if (errors & (1 << 2)) {
        printf("gyro power error\n");
      }
      if (errors & (1 << 1)) {
        printf("gyro continuous self-test error\n");
      }
      if (errors & 1) {
        printf("gyro unexpected self check mode\n");
      }
    }
    if (bad_gyro) {
      bad_reading = 1;
    }
  }
  process_reading(-(int16_t)(value >> 10 & 0xFFFF));
}

void SPI_IRQHandler(void) {
  uint32_t status = SPI->SR;
  if (status & SPI_SR_RXNE) {
    uint16_t value = SPI->DR;
    if (__builtin_parity(value) != 1) {
      parity_error = 1;
      for (int i = -2; i < 16; ++i) {
        led_set(LED_Z, i < 0);
        for (int ii = 0; ii < 1000000; ++ii) {
          led_set(LED_ERR, i >= 0 && ii < 500000);
          if (i >= 0) led_set(LED_DB, value & (1 << i));
          else led_set(LED_DB, 0);
        }
      }
    }
    if (receive_byte == 0) {
      receive_byte = 1;
      high_value = value;
    } else {
      uint32_t full_value = high_value << 16 | value;
      // Set the CSEL pin high to deselect it.
      // The parity calculation etc took long enough that this is safe now.
      CSEL_GPIO->BSRRH = 1 << CSEL_NUM;
      reading_received(full_value);
    }
  }
}

void TIM_IRQHandler(void) {
  TIM->CR1 &= ~TIM_CR1_CEN;
  TIM->SR = ~TIM_SR_CC1IF;
  switch (state) {
    case STATE_SETUP0:
      if (setup_counter++ < 100) {
        // Get it started doing a check.
        do_gyro_read(0x20000003);
      } else {
        gyro_setup_failed();
      }
      break;
    case STATE_SETUP1:  // Dummy read to clear the old latched state.
    case STATE_SETUP2:  // Read self-test data.
    case STATE_SETUP3:  // Read the second latched self-test data.
      do_gyro_read(0x20000000);
      break;
    case STATE_READ:
      ++led_flash;
      if (led_flash < kGyroReadFrequency / kFlashFrequency / 2) {
        led_set(LED_HB, 0);
      } else {
        led_set(LED_HB, 1);
      }
      if (led_flash >= kGyroReadFrequency / kFlashFrequency) {
        led_flash = 0;
      }

      if (bad_gyro) {
        led_set(LED_ERR, 1);
        printf("gyro reader giving up because of bad gyro\n");
        gyro_output.gyro_bad = 1;
        gyro_output.last_reading_bad = 1;
        gyro_output.angle = 0;
        break;
      }

      if (startup_cycles_left) {
        led_set(LED_Z, 0);
        --startup_cycles_left;
        if (bad_reading) {
          printf("gyro retrying startup wait because of bad reading\n");
          startup_cycles_left = kStartupCycles;
        }
      } else if (zeroing_cycles_left) {
        led_set(LED_Z, 1);
        --zeroing_cycles_left;
        if (bad_reading) {
          printf("gyro restarting zeroing because of bad reading\n");
          zeroing_cycles_left = kZeroingCycles;
          zero_bias = 0;
        } else {
          zero_bias -= new_reading;
          if (zeroing_cycles_left == 0) {
            // Do all the nice math
            full_units_offset = zero_bias / kZeroingCycles;
            remainder_offset = zero_bias % kZeroingCycles;
            if (remainder_offset < 0) {
              remainder_offset += kZeroingCycles;
              --full_units_offset;
            }
            gyro_output.zeroed = 1;
          }
        }
      } else {
        led_set(LED_Z, 0);

        int64_t new_angle = gyro_output.angle;
        if (!bad_reading) new_angle += new_reading + full_units_offset;
        if (remainder_sum >= kZeroingCycles) {
          remainder_sum -= kZeroingCycles;
          new_angle += 1;
        }
        gyro_output.angle = new_angle;
        gyro_output.last_reading_bad = bad_reading;
        remainder_sum += remainder_offset;
      }
      do_gyro_read(0x20000000);
      break;
  }
}

void gyro_init(void) {
  gyro_output.initialized = 0;
  gyro_output.zeroed = 0;

  RCC->APB1ENR |= RCC_APB1ENR_SPIEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIMEN;

  // Set up CSEL.
  // It's is just a GPIO pin because we're the master (it would be special if we
  // were a slave).
  gpio_setup_out(CSEL_GPIO, CSEL_NUM, 3);
  CSEL_GPIO->BSRRH = 1 << CSEL_NUM;  // make sure it's deselected

  // Set up SCK, MISO, and MOSI.
  gpio_setup_alt(GPIOC, 10, 6);  // SCK
  gpio_setup_alt(GPIOC, 11, 6);  // MISO
  gpio_setup_alt(GPIOC, 12, 6);  // MOSI

  NVIC_SetPriority(SPI_IRQn, 4);
  NVIC_EnableIRQ(SPI_IRQn);
  NVIC_SetPriority(TIM_IRQn, 5);
  NVIC_EnableIRQ(TIM_IRQn);

  TIM->CR1 = 0;
  TIM->DIER = TIM_DIER_CC1IE;
  TIM->CCMR1 = 0;
  // Make it generate 1 tick every ms.
  TIM->PSC = 60000 - 1;

  SPI->CR1 = 0;  // make sure it's disabled
  SPI->CR1 =
      SPI_CR1_DFF /* 16 bit frame */ |
      SPI_CR1_SSM | SPI_CR1_SSI | /* don't watch for other masters */
      1 << 3 /* 30MHz/4 = 7.5MHz */ |
      SPI_CR1_MSTR /* master mode */;
  SPI->CR2 = SPI_CR2_RXNEIE;
  SPI->CR1 |= SPI_CR1_SPE;  // enable it

  setup_counter = 0;
  switch_state(STATE_SETUP0, 100);
}
