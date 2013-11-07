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
#include "LPCUSB/usbapi.h"

#include "analog.h"
#include "digital.h"
#include "encoder.h"
#include "CAN.h"
#include "gyro.h"

extern void usb_init(void);

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

// Setup the peripherals.
static void setup_hardware(void) {
  // Setup GPIO power.
  SC->PCONP = PCONP_PCGPIO;

  // Disable TPIU.
  PINCON->PINSEL10 = 0;

  setup_PLL0();

  setup_PLL1();

  /* Configure the LEDs. */
  vParTestInitialise();
}

int main(void) {
  // Errata PCLKSELx.1 says that we have to do all of this BEFORE setting up
  // PLL0 or it might not work.

  // Set everything to run at full CCLK by default.
  SC->PCLKSEL0 = 0x55555555;

  CAN_PCLKSEL();

  setup_hardware();

  digital_init();

  analog_init();

  encoder_init();

  gyro_init();

  initCAN();

  usb_init();

  // Start the scheduler.
  vTaskStartScheduler();

  /* Will only get here if there was insufficient memory to create the idle
     task.  The idle task is created within vTaskStartScheduler(). */
  for (;;) {}
}

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
  /* This function will get called if a task overflows its stack. */

  (void) pxTask;
  (void) pcTaskName;

  for (;;);
}

// This is what portCONFIGURE_TIMER_FOR_RUN_TIME_STATS in FreeRTOSConfig.h
// actually calls.
// It sets up timer 0 to use for timing.
void vConfigureTimerForRunTimeStats(void) {
  const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

  /* This function configures a timer that is used as the time base when
     collecting run time statistical information - basically the percentage
     of CPU time that each task is utilising.  It is called automatically when
     the scheduler is started (assuming configGENERATE_RUN_TIME_STATS is set
     to 1). */

  /* Power up and feed the timer. */
  SC->PCONP |= 0x02UL;

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
