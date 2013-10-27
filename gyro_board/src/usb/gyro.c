#include "gyro.h"

#include <stdio.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

struct GyroOutput gyro_output;

static void gyro_disable_csel(void) {
  // Set the CSEL pin high to deselect it.
  GPIO0->FIOSET = 1 << 16;
}

static void gyro_enable_csel(void) {
  // Clear the CSEL pin to select it.
  GPIO0->FIOCLR = 1 << 16;
}

// Blocks until there is data available.
static uint16_t spi_read(void) {
  while (!(SSP0->SR & (1 << 2))) {}
  return SSP0->DR;
}

// Blocks until there is space to enqueue data.
static void spi_write(uint16_t data) {
  while (!(SSP0->SR & (1 << 1))) {}
  SSP0->DR = data;
}

static uint32_t do_gyro_read(uint32_t data, int *parity_error) {
  *parity_error = 0;

  gyro_enable_csel();
  spi_write(data >> 16);
  if (__builtin_parity(data & ~1) == 0) data |= 1;
  spi_write(data);

  uint16_t high_value = spi_read();
  if (__builtin_parity(high_value) != 1) {
    printf("high value 0x%"PRIx16" parity error\n", high_value);
    *parity_error = 1;
  }
  uint16_t low_value = spi_read();
  gyro_disable_csel();
  uint32_t r = high_value << 16 | low_value;
  if (__builtin_parity(r) != 1) {
    printf("low value 0x%"PRIx16" parity error (r=%"PRIx32")\n", low_value, r);
    *parity_error = 1;
  }

  return r;
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

// Performs a read from the gyro.
// Sets *bad_reading to 1 if the result is potentially bad and *bad_gyro to 1 if
// the gyro is bad and we're not going to get any more readings.
static int16_t gyro_read(int *bad_reading, int *bad_gyro) {
  *bad_reading = *bad_gyro = 0;

  int parity_error;
  uint32_t value = do_gyro_read(0x20000000, &parity_error);

  if (parity_error) {
    *bad_reading = 1;
    return 0;
  }

  // This check assumes that the sequence bits are all 0, but they should be
  // because that's all we send.
  if (gyro_status(value) != 1) {
    uint8_t status = gyro_status(value);
    if (status == 0) {
      printf("gyro says sensor data is bad\n");
    } else {
      printf("gyro gave weird status 0x%"PRIx8"\n", status);
    }
    *bad_reading = 1;
  }

  if (gyro_errors(value) != 0) {
    uint8_t errors = gyro_errors(value);
    if (errors & ~(1 << 1)) {
      *bad_reading = 1;
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
      *bad_gyro = 1;
    }
    if (errors & (1 << 3)) {
      printf("gyro volatile memory error\n");
      *bad_gyro = 1;
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
  if (*bad_gyro) {
    *bad_reading = 1;
    return 0;
  } else {
    return -(int16_t)(value >> 10 & 0xFFFF);
  }
}

// Returns 1 if the setup failed or 0 if it succeeded.
static int gyro_setup(void) {
  for (int i = 0; i < 100; ++i) {
	  portTickType wait_time = xTaskGetTickCount();
    int parity_error;

    // Wait for it to start up.
	  vTaskDelayUntil(&wait_time, 100 / portTICK_RATE_MS);
    // Get it started doing a check.
    uint32_t value = do_gyro_read(0x20000003, &parity_error);
    if (parity_error) continue;
    // Its initial response is hardcoded to 1.
    if (value != 1) {
      printf("gyro unexpected initial response 0x%"PRIx32"\n", value);
      // There's a chance that we're retrying because of a parity error
      // previously, so keep going.
    }

    // Wait for it to assert the fault conditions.
	  vTaskDelayUntil(&wait_time, 50 / portTICK_RATE_MS);
    // Dummy read to clear the old latched state.
    do_gyro_read(0x20000000, &parity_error);
    if (parity_error) continue;

    // Wait for it to clear the fault conditions.
	  vTaskDelayUntil(&wait_time, 50 / portTICK_RATE_MS);
    value = do_gyro_read(0x20000000, &parity_error);
    if (parity_error) continue;
    // If it's not reporting self test data.
    if (gyro_status(value) != 2) {
      printf("gyro first value 0x%"PRIx32" not self test data\n", value);
      continue;
    }
    // If we don't see all of the errors.
    if (gyro_errors(value) != 0x7F) {
      printf("gyro self test value 0x%"PRIx32" is bad\n", value);
      return 1;
    }

    // Wait for the sequential transfer delay.
	  vTaskDelayUntil(&wait_time, 1 / portTICK_RATE_MS);
    value = do_gyro_read(0x20000000, &parity_error);
    if (parity_error) continue;
    // It should still be reporting self test data.
    if (gyro_status(value) != 2) {
      printf("gyro second value 0x%"PRIx32" not self test data\n", value);
      continue;
    }
    return 0;
  }
  return 1;
}

static portTASK_FUNCTION(gyro_read_task, pvParameters) {
  SC->PCONP |= PCONP_PCSSP0;

  // Make sure that the clock is set up right.
  SC->PCLKSEL1 &= ~(3 << 10);
  SC->PCLKSEL1 |= 1 << 10;

  // Set up SSEL.
  // It's is just a GPIO pin because we're the master (it would be special if we
  // were a slave).
  gyro_disable_csel();
  GPIO0->FIODIR |= 1 << 16;
  PINCON->PINSEL1 &= ~(3 << 0);
  PINCON->PINSEL1 |= 0 << 0;

  // Set up MISO0 and MOSI0.
  PINCON->PINSEL1 &= ~(3 << 2 | 3 << 4);
  PINCON->PINSEL1 |= 2 << 2 | 2 << 4;

  // Set up SCK0.
  PINCON->PINSEL0 &= ~(3 << 30);
  PINCON->PINSEL0 |= (2 << 30);

  // Make sure it's disabled.
  SSP0->CR1 = 0;
  SSP0->CR0 =
      0xF /* 16 bit transfer */ |
      0 << 4 /* SPI mode */ |
      0 << 6 /* CPOL = 0 */ |
      0 << 7 /* CPHA = 0 */;
  // 14 clocks per cycle.  This works out to a ~7.2MHz bus.
  // The gyro is rated for a maximum of 8.08MHz.
  SSP0->CPSR = 14;
  // Finally, enable it.
  // This has to be done after we're done messing with everything else.
  SSP0->CR1 |= 1 << 1;

  if (gyro_setup()) {
    printf("gyro setup failed. deleting task\n");
    gyro_output.angle = 0;
    gyro_output.last_reading_bad = gyro_output.gyro_bad = 1;
    gyro_output.initialized = 1;
    vTaskDelete(NULL);
    return;
  } else {
    gyro_output.initialized = 1;
  }

  gyro_output.angle = 0;
  gyro_output.last_reading_bad = 1;  // until we're started up
  gyro_output.gyro_bad = 0;

  // How many times per second to read the gyro value.
  static const int kGyroReadFrequency = 200;
  // How many times per second to flash the LED.
  // Must evenly divide kGyroReadFrequency.
  static const int kFlashFrequency = 10;

  static const int kStartupCycles = kGyroReadFrequency * 2;
  static const int kZeroingCycles = kGyroReadFrequency * 6;

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
  vParTestSetLED(0, 0);

  portTickType xLastGyroReadTime = xTaskGetTickCount();

  for (;;) {
    ++led_flash;
    if (led_flash < kGyroReadFrequency / kFlashFrequency / 2) {
      vParTestSetLED(1, 0);
    } else {
      vParTestSetLED(1, 1);
    }
    if (led_flash >= kGyroReadFrequency / kFlashFrequency) {
      led_flash = 0;
    }

    vTaskDelayUntil(&xLastGyroReadTime,
                    1000 / kGyroReadFrequency / portTICK_RATE_MS);

    int bad_reading, bad_gyro;
    int16_t gyro_value = gyro_read(&bad_reading, &bad_gyro);
    if (bad_gyro) {
      // We're just going to give up if this happens (write out that we're
      // giving up and then never run anything else in this task).
      vParTestSetLED(0, 1);
      printf("gyro read task giving up because of bad gyro\n");
      portENTER_CRITICAL();
      gyro_output.gyro_bad = 1;
      gyro_output.last_reading_bad = 1;
      gyro_output.angle = 0;
      portEXIT_CRITICAL();
      vTaskDelete(NULL);
      while (1) {}
    }

    if (startup_cycles_left) {
      vParTestSetLED(2, 0);
      --startup_cycles_left;
      if (bad_reading) {
        printf("gyro retrying startup wait because of bad reading\n");
        startup_cycles_left = kStartupCycles;
      }
    } else if (zeroing_cycles_left) {
      vParTestSetLED(2, 1);
      --zeroing_cycles_left;
      if (bad_reading) {
        printf("gyro restarting zeroing because of bad reading\n");
        zeroing_cycles_left = kZeroingCycles;
        zero_bias = 0;
      } else {
        zero_bias -= gyro_value;
        if (zeroing_cycles_left == 0) {
          // Do all the nice math
          full_units_offset = zero_bias / kZeroingCycles;
          remainder_offset = zero_bias % kZeroingCycles;
          if (remainder_offset < 0) {
            remainder_offset += kZeroingCycles;
            --full_units_offset;
          }
        }
      }
    } else {
      vParTestSetLED(2, 0);

      int64_t new_angle = gyro_output.angle;
      if (!bad_reading) new_angle += gyro_value + full_units_offset;
      if (remainder_sum >= kZeroingCycles) {
        remainder_sum -= kZeroingCycles;
        new_angle += 1;
      }
      portENTER_CRITICAL();
      gyro_output.angle = new_angle;
      gyro_output.last_reading_bad = bad_reading;
      portEXIT_CRITICAL();
      remainder_sum += remainder_offset;
    }
  }
}

void gyro_init(void) {
  gyro_output.initialized = 0;

  xTaskCreate(gyro_read_task, (signed char *) "gyro",
              configMINIMAL_STACK_SIZE + 100, NULL,
              tskIDLE_PRIORITY + 2, NULL);
}
