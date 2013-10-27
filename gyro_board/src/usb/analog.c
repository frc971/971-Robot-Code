// ****************************************************************************
// CopyLeft qwerk Robotics unINC. 2010 All Rights Reserved.
// ****************************************************************************

#include "analog.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

void analog_init(void) {
  // b[1:0] CAN RD1 p0.0
  // b[3:2] CAN TD1 p0.1
  //PINCON->PINSEL0 = 0x00000005;

  // b[29:28] USB_DMIN   p0.30
  // b[27:26] USB_DPLUS  p0.29
  // b[21:20] AD0.3  p0.26
  // b[19:18] AD0.2  p0.25
  // PINCON->PINSEL1 = 0x14140000;

  // PINCON->PINSEL2 = 0x0;

  // b[31:30] AD0.5  p1.31
  // b[29:28] V_BUS  p1.30
  // b[21:20] MCOB1  p1.26
  // b[19:18] MCOA1  p1.25
  // b[15:14] MCI1  p1.23
  // b[13:12] MCOB0  p1.22
  // b[09:08] MCI0  p1.20
  // b[07:06] MCOA0  p1.19
  // b[05:04] USB_UP_LED  p1.18
  //PINCON->PINSEL3 = 0xE0145150;
  SC->PCONP |= PCONP_PCAD;

  // Enable AD0.0, AD0.1, AD0.2, AD0.3
  PINCON->PINSEL1 &= 0xFFC03FFF;
  PINCON->PINSEL1 |= 0x00D54000;
  ADC->ADCR = 0x00200500;
}


int analog(int channel) {
  ADC->ADCR = ((ADC->ADCR & 0xF8FFFF00) | (0x01000000 | (1 << channel)));

  // Poll until it is done.
  while(!(ADC->ADGDR & 0x80000000));

  return ((ADC->ADGDR & 0x0000FFF0) >> 4);
}
