/*
    FreeRTOS V6.0.5 - Copyright (C) 2010 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

*/

/* Standard includes. */
#include "stdio.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo app includes. */
#include "flash.h"
#include "partest.h"
#include "spi.h"
#include "LPCUSB/usbapi.h"

#include "analog.h"
#include "digital.h"
#include "encoder.h"
#include "CAN.h"

int64_t gyro_angle = 0;

/*
 * Configure the hardware.
 */
static void prvSetupHardware(void);

/*
 * The task that handles the USB stack.
 */
extern void vUSBTask(void *pvParameters);

extern int VCOM_getchar(void);

int VCOM_putchar(int c);

inline int32_t encoder()
{
  return (int32_t)QEI->QEIPOS;
}

static portTASK_FUNCTION(vPrintPeriodic, pvParameters)
{
  portTickType xLastFlashTime;

  /* We need to initialise xLastFlashTime prior to the first call to
     vTaskDelayUntil(). */
  xLastFlashTime = xTaskGetTickCount();

  digital_init();

  analog_init();

  encoder_init();

  // Wait 100 ms for it to boot.
  vTaskDelayUntil(&xLastFlashTime, 100 / portTICK_RATE_MS);
  spi_init();

  // Enable USB.  The PC has probably disconnected it now.
  USBHwAllowConnect();

  // TODO(aschuh): Write this into a gyro calibration function, and check all the outputs.
  vTaskDelayUntil(&xLastFlashTime, 50 / portTICK_RATE_MS);
  enable_gyro_csel();
  printf("SPI Gyro Second Response 0x%x %x\n", transfer_spi_bytes(0x2000), transfer_spi_bytes(0x0000));
  disable_gyro_csel();

  vTaskDelayUntil(&xLastFlashTime, 50 / portTICK_RATE_MS);
  enable_gyro_csel();
  printf("SPI Gyro Third Response 0x%x %x\n", transfer_spi_bytes(0x2000), transfer_spi_bytes(0x0000));
  disable_gyro_csel();

  vTaskDelayUntil(&xLastFlashTime, 10 / portTICK_RATE_MS);
  enable_gyro_csel();
  printf("SPI Gyro Fourth Response 0x%x %x\n", transfer_spi_bytes(0x2000), transfer_spi_bytes(0x0000));
  disable_gyro_csel();
  const int hz = 200;
  const int flash_hz = 10;
  const int startup_cycles = hz * 2;
  const int zeroing_cycles = hz * 6;
  int32_t zero_bias = 0;
  int32_t startup_cycles_left = startup_cycles;
  int32_t zeroing_cycles_left = zeroing_cycles;
  int32_t full_units_offset = 0;
  int32_t remainder_offset = 0;
  int32_t remainder_sum = 0;
  int32_t led_flash = 0;
  vParTestSetLED(0, 0);

  for (;;) {
    led_flash ++;
    if (led_flash < hz / flash_hz / 2) {
      vParTestSetLED(1, 0);
    } else {
      vParTestSetLED(1, 1);
    }
    if (led_flash >= hz / flash_hz) {
      led_flash = 0;
    }
    /* Delay for half the flash period then turn the LED on. */
    vTaskDelayUntil(&xLastFlashTime, 1000 / hz / portTICK_RATE_MS);
    enable_gyro_csel();
    uint16_t high_value = transfer_spi_bytes(0x2000);
    uint16_t low_value = transfer_spi_bytes(0x0000);
    disable_gyro_csel();
    int16_t gyro_value = -((int16_t)((((uint32_t)high_value << 16) | (uint32_t)low_value) >> 10));

    if (startup_cycles_left) {
      vParTestSetLED(2, 0);
      --startup_cycles_left;
    } else if (zeroing_cycles_left) {
      vParTestSetLED(2, 1);
      //printf("Zeroing ");
      --zeroing_cycles_left;
      zero_bias -= gyro_value;
      if (zeroing_cycles_left == 0) {
        // Do all the nice math
        full_units_offset = zero_bias / zeroing_cycles;
        remainder_offset = zero_bias % zeroing_cycles;
        if (remainder_offset < 0) {
          remainder_offset += zeroing_cycles;
          --full_units_offset;
        }
      }
    } else {
      vParTestSetLED(2, 0);
      int64_t new_angle = gyro_angle + gyro_value + full_units_offset;
      if (remainder_sum >= zeroing_cycles) {
        remainder_sum -= zeroing_cycles;
        new_angle += 1;
      }
      gyro_angle = new_angle;
      remainder_sum += remainder_offset;
    }
    //printf("Angle %d Rate %d\n", (int)(gyro_angle / 16),
    //       (int)(gyro_value + full_units_offset));

    //printf("time: %d analog %d encoder %d goal %d\n", (int)i, (int)analog(5),
    //       (int)encoder(), (int)goal);

    /*
    for(i = 0; i < 4; i++){
      printf("analog(%d) => %d\n",i,analog(i));
    }
    for(i = 1; i < 13; i++){
      printf("digital(%d) => %d\n",i,digital(i));
    }
    for(i = 0; i < 4; i++){
      printf("dip(%d) => %d\n",i,dip(i));
    }
    for(i = 0; i < 4; i++){
      printf("encoder(%d) => %d\n",i,encoder_bits(i));
    }
    for(i = 0; i < 4; i++){
      printf("encoder_val(%d) => %d\n",i,(int)encoder_val(i));
    }*/
  }
}

int main(void) {
  // Configure the hardware
  prvSetupHardware();

  /* Create the USB task. */
  xTaskCreate(vUSBTask, (signed char *) "USB",
              configMINIMAL_STACK_SIZE + 1020, (void *) NULL,
              tskIDLE_PRIORITY + 3, NULL);

  xTaskCreate(vPrintPeriodic, (signed char *) "PRINTx",
              configMINIMAL_STACK_SIZE + 100, NULL,
              tskIDLE_PRIORITY + 2, NULL);


  initCAN();

  // Start the scheduler.
  vTaskStartScheduler();

  /* Will only get here if there was insufficient memory to create the idle
     task.  The idle task is created within vTaskStartScheduler(). */
  for (;;) {}
}
/*-----------------------------------------------------------*/

// Sets up (and connects) PLL0.
// The CPU will be running at 100 MHz with a 12 MHz clock input when this is
// done.
static void setup_PLL0(void) {
  // If PLL0 is currently connected.
  if (SC->PLL0STAT & (1 << 25)) {
    /* Enable PLL0, disconnected. */
    SC->PLL0CON = 1;
    SC->PLL0FEED = PLLFEED_FEED1;
    SC->PLL0FEED = PLLFEED_FEED2;
  }

  // Disable PLL0, disconnected.
  SC->PLL0CON = 0;
  SC->PLL0FEED = PLLFEED_FEED1;
  SC->PLL0FEED = PLLFEED_FEED2;

  // Enable main OSC.
  SC->SCS |= 1 << 5;
  // Wait until it's ready.
  while (!(SC->SCS & (1 << 6)));

  // Select main OSC as the PLL0 clock source.
  SC->CLKSRCSEL = 0x1;

  // Set up PLL0 to output 400MHz.
  // 12MHz * 50 / 3 * 2 = 400MHz.
  // The input is 12MHz (from the crystal), M = 50, and N = 3.
  SC->PLL0CFG = 0x20031;
  SC->PLL0FEED = PLLFEED_FEED1;
  SC->PLL0FEED = PLLFEED_FEED2;

  // Enable PLL0, disconnected.
  SC->PLL0CON = 1;
  SC->PLL0FEED = PLLFEED_FEED1;
  SC->PLL0FEED = PLLFEED_FEED2;

  // Set clock divider to dividing by 4 to give a final frequency of 100MHz.
  SC->CCLKCFG = 0x03;

  // Configure flash accelerator to use 5 CPU clocks like the datasheet says for
  // a 100MHz clock.
  SC->FLASHCFG = 0x403a;

  // Wait until PLL0 is locked.
  while (((SC->PLL0STAT & (1 << 26)) == 0));

  // Enable PLL0 and connect.
  SC->PLL0CON = 3;
  SC->PLL0FEED = PLLFEED_FEED1;
  SC->PLL0FEED = PLLFEED_FEED2;

  // Wait until PLL0 is connected.
  while (((SC->PLL0STAT & (1 << 25)) == 0));
}

// Configures PLL1 as the clock for USB.
static void setup_PLL1(void) {
  // If PLL1 is currently connected.
  if (SC->PLL1STAT & (1 << 9)) {
    // Enable PLL1, disconnected.
    SC->PLL1CON = 1;
    SC->PLL1FEED = PLLFEED_FEED1;
    SC->PLL1FEED = PLLFEED_FEED2;
  }

  // Disable PLL1, disconnected.
  SC->PLL1CON = 0;
  SC->PLL1FEED = PLLFEED_FEED1;
  SC->PLL1FEED = PLLFEED_FEED2;

  // Set up PLL1 to produce the required 48MHz from the crystal.
  // USBCLK = 12MHz * 4 = 48MHz.
  // FCCO = USBCLK * 2 * 3 = 288MHz.
  // The input is 12MHz, M = 4, and P = 3.
  SC->PLL1CFG = 0x23;
  SC->PLL1FEED = PLLFEED_FEED1;
  SC->PLL1FEED = PLLFEED_FEED2;

  /* Enable PLL1, disconnected. */
  SC->PLL1CON = 1;
  SC->PLL1FEED = PLLFEED_FEED1;
  SC->PLL1FEED = PLLFEED_FEED2;

  // Wait until PLL1 is locked.
  while (((SC->PLL1STAT & (1 << 10)) == 0));

  /* Enable PLL1 and connect it. */
  SC->PLL1CON = 3;
  SC->PLL1FEED = PLLFEED_FEED1;
  SC->PLL1FEED = PLLFEED_FEED2;

  // Wait until PLL1 is connected.
  while (((SC->PLL1STAT & (1 << 9)) == 0));
}

void prvSetupHardware(void)
{
  // Setup the peripherals.

  // Setup GPIO power.
  SC->PCONP = PCONP_PCGPIO;

  // Disable TPIU.
  PINCON->PINSEL10 = 0;

  setup_PLL0();

  setup_PLL1();

  // Setup the peripheral bus to be the same as the CCLK, 100 MHz.
  // Set CAN to run at CCLK/6, which should have it running about 1 Mbit (1.042)
  SC->PCLKSEL0 = 0xff555555;

  /* Configure the LEDs. */
  vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
  /* This function will get called if a task overflows its stack. */

  (void) pxTask;
  (void) pcTaskName;

  for (;;);
}
/*-----------------------------------------------------------*/

// This is what portCONFIGURE_TIMER_FOR_RUN_TIME_STATS in FreeRTOSConfig.h
// actually calls.
void vConfigureTimerForRunTimeStats(void)
{
  const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

  /* This function configures a timer that is used as the time base when
     collecting run time statistical information - basically the percentage
     of CPU time that each task is utilising.  It is called automatically when
     the scheduler is started (assuming configGENERATE_RUN_TIME_STATS is set
     to 1). */

  /* Power up and feed the timer. */
  SC->PCONP |= 0x02UL;
  SC->PCLKSEL0 = (SC->PCLKSEL0 & (~(0x3 << 2))) | (0x01 << 2);

  /* Reset Timer 0 */
  TIM0->TCR = TCR_COUNT_RESET;

  /* Just count up. */
  TIM0->CTCR = CTCR_CTM_TIMER;

  /* Prescale to a frequency that is good enough to get a decent resolution,
     but not too fast so as to overflow all the time. */
  TIM0->PR = (configCPU_CLOCK_HZ / 10000UL) - 1UL;

  /* Start the counter. */
  TIM0->TCR = TCR_COUNT_ENABLE;
}
