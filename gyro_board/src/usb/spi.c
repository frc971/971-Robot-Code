#include "stdio.h"
#include "FreeRTOS.h"
#include "spi.h"

void spi_init (void) {
  SC->PCONP |= PCONP_PCSPI;
  SC->PCLKSEL0 |= 0x00010000;

  // Hook up the interrupt
  //NVIC_EnableIRQ(SPI_IRQn);

  // SCK
  PINCON->PINSEL0 &= 0x3fffffff;
  PINCON->PINSEL0 |= 0xc0000000;

  // SSEL, MISO, MOSI
  // SSEL is GPIO, and needs to be done manually.
  disable_gyro_csel();
  GPIO0->FIODIR |= 0x00010000;
  PINCON->PINSEL1 &= 0xffffffc0;
  PINCON->PINSEL1 |= 0x0000003c;

  // Master mode, 16 bits/frame, enable interrupts
  SPI->SPCR = 0x000000a4;
  // 13 clocks per cycle.  This works out to a 7.7 mhz buss.
  SPI->SPCCR = 0x0000000d;

  // TODO(aschuh): Implement the gyro bring-up blocking first.
  // Then use interrupts.
  enable_gyro_csel();
  printf("SPI Gyro Initial Response 0x%x %x\n", transfer_spi_bytes(0x2000), transfer_spi_bytes(0x0003));
  disable_gyro_csel();
}

// TODO: DMA? SSP0?  SSP0 should have a buffer, which would be very nice.
uint16_t transfer_spi_bytes(uint16_t data) {
  SPI->SPDR = (uint32_t)data;
  while (!(SPI->SPSR & 0x80));
  return SPI->SPDR;
}

void disable_gyro_csel (void) {
  // Set the CSEL pin high to deselect it.
  GPIO0->FIOSET = 0x00010000;
}

void enable_gyro_csel (void) {
  // Clear the CSEL pin high to select it.
  GPIO0->FIOCLR = 0x00010000;
}

void SPI_IRQHandler(void) {
  int status = SPI->SPSR;
  if (status & 0x80) {
    // Transfer completed.
  }

  // Clear the interrupt?
  SPI->SPINT = 0x00000001;
}
