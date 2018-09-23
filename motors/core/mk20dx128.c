/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "motors/core/kinetis.h"

#include <errno.h>
#include <stdio.h>

#define FSEC                                   \
  ((2 << 6) /* Enable backdoor key access */ | \
   (1 << 4) /* Enable mass erase */ |          \
   (3 << 2) /* Freescale access granted */ | (2 << 0) /* Not secured */)

#define FOPT                                                            \
  ((0 << 2) /* NMI always blocked */ | (0 << 1) /* EzPort disabled */ | \
   (1 << 0) /* Normal (not low-power) boot */)

extern uint32_t __heap_start__[];
extern uint32_t __heap_end__[];
extern uint32_t __stack_end__[];

extern int main(void);
void ResetHandler(void);
void __libc_init_array(void);

void fault_isr(void) {
  FTM0_C0V = FTM0_C1V = FTM0_C2V = FTM0_C3V = FTM0_C4V = FTM0_C5V = 0;
  FTM3_C0V = FTM3_C1V = FTM3_C2V = FTM3_C3V = FTM3_C4V = FTM3_C5V = 0;
  printf("fault_isr\n");
  while (1) {
    // keep polling some communication while in fault
    // mode, so we don't completely die.
    if (SIM_SCGC4 & SIM_SCGC4_USBOTG) usb_isr();
  }
}

void unused_isr(void) { fault_isr(); }

extern volatile uint32_t systick_millis_count;

void nmi_isr(void) __attribute__((weak, alias("unused_isr")));
void hard_fault_isr(void) __attribute__((weak, alias("fault_isr")));
void memmanage_fault_isr(void) __attribute__((weak, alias("fault_isr")));
void bus_fault_isr(void) __attribute__((weak, alias("fault_isr")));
void usage_fault_isr(void) __attribute__((weak, alias("fault_isr")));
void svcall_isr(void) __attribute__((weak, alias("unused_isr")));
void debugmonitor_isr(void) __attribute__((weak, alias("unused_isr")));
void pendablesrvreq_isr(void) __attribute__((weak, alias("unused_isr")));
void systick_isr(void) { systick_millis_count++; }

void dma_ch0_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch1_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch2_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch3_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch4_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch5_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch6_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch7_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch8_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch9_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch10_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch11_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch12_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch13_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch14_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_ch15_isr(void) __attribute__((weak, alias("unused_isr")));
void dma_error_isr(void) __attribute__((weak, alias("unused_isr")));
void mcm_isr(void) __attribute__((weak, alias("unused_isr")));
void randnum_isr(void) __attribute__((weak, alias("unused_isr")));
void flash_cmd_isr(void) __attribute__((weak, alias("unused_isr")));
void flash_error_isr(void) __attribute__((weak, alias("unused_isr")));
void low_voltage_isr(void) __attribute__((weak, alias("unused_isr")));
void wakeup_isr(void) __attribute__((weak, alias("unused_isr")));
void watchdog_isr(void) __attribute__((weak, alias("unused_isr")));
void i2c0_isr(void) __attribute__((weak, alias("unused_isr")));
void i2c1_isr(void) __attribute__((weak, alias("unused_isr")));
void i2c2_isr(void) __attribute__((weak, alias("unused_isr")));
void i2c3_isr(void) __attribute__((weak, alias("unused_isr")));
void spi0_isr(void) __attribute__((weak, alias("unused_isr")));
void spi1_isr(void) __attribute__((weak, alias("unused_isr")));
void spi2_isr(void) __attribute__((weak, alias("unused_isr")));
void sdhc_isr(void) __attribute__((weak, alias("unused_isr")));
void enet_timer_isr(void) __attribute__((weak, alias("unused_isr")));
void enet_tx_isr(void) __attribute__((weak, alias("unused_isr")));
void enet_rx_isr(void) __attribute__((weak, alias("unused_isr")));
void enet_error_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_message_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_bus_off_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_error_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_tx_warn_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_rx_warn_isr(void) __attribute__((weak, alias("unused_isr")));
void can0_wakeup_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_message_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_bus_off_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_error_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_tx_warn_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_rx_warn_isr(void) __attribute__((weak, alias("unused_isr")));
void can1_wakeup_isr(void) __attribute__((weak, alias("unused_isr")));
void i2s0_tx_isr(void) __attribute__((weak, alias("unused_isr")));
void i2s0_rx_isr(void) __attribute__((weak, alias("unused_isr")));
void i2s0_isr(void) __attribute__((weak, alias("unused_isr")));
void uart0_lon_isr(void) __attribute__((weak, alias("unused_isr")));
void uart0_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart0_error_isr(void) __attribute__((weak, alias("unused_isr")));
void uart1_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart1_error_isr(void) __attribute__((weak, alias("unused_isr")));
void uart2_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart2_error_isr(void) __attribute__((weak, alias("unused_isr")));
void uart3_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart3_error_isr(void) __attribute__((weak, alias("unused_isr")));
void uart4_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart4_error_isr(void) __attribute__((weak, alias("unused_isr")));
void uart5_status_isr(void) __attribute__((weak, alias("unused_isr")));
void uart5_error_isr(void) __attribute__((weak, alias("unused_isr")));
void lpuart0_status_isr(void) __attribute__((weak, alias("unused_isr")));
void adc0_isr(void) __attribute__((weak, alias("unused_isr")));
void adc1_isr(void) __attribute__((weak, alias("unused_isr")));
void cmp0_isr(void) __attribute__((weak, alias("unused_isr")));
void cmp1_isr(void) __attribute__((weak, alias("unused_isr")));
void cmp2_isr(void) __attribute__((weak, alias("unused_isr")));
void cmp3_isr(void) __attribute__((weak, alias("unused_isr")));
void ftm0_isr(void) __attribute__((weak, alias("unused_isr")));
void ftm1_isr(void) __attribute__((weak, alias("unused_isr")));
void ftm2_isr(void) __attribute__((weak, alias("unused_isr")));
void ftm3_isr(void) __attribute__((weak, alias("unused_isr")));
void tpm0_isr(void) __attribute__((weak, alias("unused_isr")));
void tpm1_isr(void) __attribute__((weak, alias("unused_isr")));
void tpm2_isr(void) __attribute__((weak, alias("unused_isr")));
void cmt_isr(void) __attribute__((weak, alias("unused_isr")));
void rtc_alarm_isr(void) __attribute__((weak, alias("unused_isr")));
void rtc_seconds_isr(void) __attribute__((weak, alias("unused_isr")));
void pit_isr(void) __attribute__((weak, alias("unused_isr")));
void pit0_isr(void) __attribute__((weak, alias("unused_isr")));
void pit1_isr(void) __attribute__((weak, alias("unused_isr")));
void pit2_isr(void) __attribute__((weak, alias("unused_isr")));
void pit3_isr(void) __attribute__((weak, alias("unused_isr")));
void pdb_isr(void) __attribute__((weak, alias("unused_isr")));
void usb_isr(void) __attribute__((weak, alias("unused_isr")));
void usb_charge_isr(void) __attribute__((weak, alias("unused_isr")));
void usbhs_isr(void) __attribute__((weak, alias("unused_isr")));
void usbhs_phy_isr(void) __attribute__((weak, alias("unused_isr")));
void dac0_isr(void) __attribute__((weak, alias("unused_isr")));
void dac1_isr(void) __attribute__((weak, alias("unused_isr")));
void tsi0_isr(void) __attribute__((weak, alias("unused_isr")));
void mcg_isr(void) __attribute__((weak, alias("unused_isr")));
void lptmr_isr(void) __attribute__((weak, alias("unused_isr")));
void porta_isr(void) __attribute__((weak, alias("unused_isr")));
void portb_isr(void) __attribute__((weak, alias("unused_isr")));
void portc_isr(void) __attribute__((weak, alias("unused_isr")));
void portd_isr(void) __attribute__((weak, alias("unused_isr")));
void porte_isr(void) __attribute__((weak, alias("unused_isr")));
void portcd_isr(void) __attribute__((weak, alias("unused_isr")));
void software_isr(void) __attribute__((weak, alias("unused_isr")));

__attribute__((used, aligned(512))) void (
    *_VectorsRam[NVIC_NUM_INTERRUPTS + 16])(void);

__attribute__((section(".vectors"), used)) void (
    *const _VectorsFlash[NVIC_NUM_INTERRUPTS + 16])(void) = {
    (void (*)(void))((unsigned long)
                     __stack_end__),  //  0 ARM: Initial Stack Pointer
    ResetHandler,                     //  1 ARM: Initial Program Counter
    nmi_isr,                          //  2 ARM: Non-maskable Interrupt (NMI)
    hard_fault_isr,                   //  3 ARM: Hard Fault
    memmanage_fault_isr,              //  4 ARM: MemManage Fault
    bus_fault_isr,                    //  5 ARM: Bus Fault
    usage_fault_isr,                  //  6 ARM: Usage Fault
    fault_isr,                        //  7 --
    fault_isr,                        //  8 --
    fault_isr,                        //  9 --
    fault_isr,                        // 10 --
    svcall_isr,                       // 11 ARM: Supervisor call (SVCall)
    debugmonitor_isr,                 // 12 ARM: Debug Monitor
    fault_isr,                        // 13 --
    pendablesrvreq_isr,  // 14 ARM: Pendable req serv(PendableSrvReq)
    systick_isr,         // 15 ARM: System tick timer (SysTick)
#if defined(__MK20DX256__)
    dma_ch0_isr,       // 16 DMA channel 0 transfer complete
    dma_ch1_isr,       // 17 DMA channel 1 transfer complete
    dma_ch2_isr,       // 18 DMA channel 2 transfer complete
    dma_ch3_isr,       // 19 DMA channel 3 transfer complete
    dma_ch4_isr,       // 20 DMA channel 4 transfer complete
    dma_ch5_isr,       // 21 DMA channel 5 transfer complete
    dma_ch6_isr,       // 22 DMA channel 6 transfer complete
    dma_ch7_isr,       // 23 DMA channel 7 transfer complete
    dma_ch8_isr,       // 24 DMA channel 8 transfer complete
    dma_ch9_isr,       // 25 DMA channel 9 transfer complete
    dma_ch10_isr,      // 26 DMA channel 10 transfer complete
    dma_ch11_isr,      // 27 DMA channel 11 transfer complete
    dma_ch12_isr,      // 28 DMA channel 12 transfer complete
    dma_ch13_isr,      // 29 DMA channel 13 transfer complete
    dma_ch14_isr,      // 30 DMA channel 14 transfer complete
    dma_ch15_isr,      // 31 DMA channel 15 transfer complete
    dma_error_isr,     // 32 DMA error interrupt channel
    unused_isr,        // 33 --
    flash_cmd_isr,     // 34 Flash Memory Command complete
    flash_error_isr,   // 35 Flash Read collision
    low_voltage_isr,   // 36 Low-voltage detect/warning
    wakeup_isr,        // 37 Low Leakage Wakeup
    watchdog_isr,      // 38 Both EWM and WDOG interrupt
    unused_isr,        // 39 --
    i2c0_isr,          // 40 I2C0
    i2c1_isr,          // 41 I2C1
    spi0_isr,          // 42 SPI0
    spi1_isr,          // 43 SPI1
    unused_isr,        // 44 --
    can0_message_isr,  // 45 CAN OR'ed Message buffer (0-15)
    can0_bus_off_isr,  // 46 CAN Bus Off
    can0_error_isr,    // 47 CAN Error
    can0_tx_warn_isr,  // 48 CAN Transmit Warning
    can0_rx_warn_isr,  // 49 CAN Receive Warning
    can0_wakeup_isr,   // 50 CAN Wake Up
    i2s0_tx_isr,       // 51 I2S0 Transmit
    i2s0_rx_isr,       // 52 I2S0 Receive
    unused_isr,        // 53 --
    unused_isr,        // 54 --
    unused_isr,        // 55 --
    unused_isr,        // 56 --
    unused_isr,        // 57 --
    unused_isr,        // 58 --
    unused_isr,        // 59 --
    uart0_lon_isr,     // 60 UART0 CEA709.1-B (LON) status
    uart0_status_isr,  // 61 UART0 status
    uart0_error_isr,   // 62 UART0 error
    uart1_status_isr,  // 63 UART1 status
    uart1_error_isr,   // 64 UART1 error
    uart2_status_isr,  // 65 UART2 status
    uart2_error_isr,   // 66 UART2 error
    unused_isr,        // 67 --
    unused_isr,        // 68 --
    unused_isr,        // 69 --
    unused_isr,        // 70 --
    unused_isr,        // 71 --
    unused_isr,        // 72 --
    adc0_isr,          // 73 ADC0
    adc1_isr,          // 74 ADC1
    cmp0_isr,          // 75 CMP0
    cmp1_isr,          // 76 CMP1
    cmp2_isr,          // 77 CMP2
    ftm0_isr,          // 78 FTM0
    ftm1_isr,          // 79 FTM1
    ftm2_isr,          // 80 FTM2
    cmt_isr,           // 81 CMT
    rtc_alarm_isr,     // 82 RTC Alarm interrupt
    rtc_seconds_isr,   // 83 RTC Seconds interrupt
    pit0_isr,          // 84 PIT Channel 0
    pit1_isr,          // 85 PIT Channel 1
    pit2_isr,          // 86 PIT Channel 2
    pit3_isr,          // 87 PIT Channel 3
    pdb_isr,           // 88 PDB Programmable Delay Block
    usb_isr,           // 89 USB OTG
    usb_charge_isr,    // 90 USB Charger Detect
    unused_isr,        // 91 --
    unused_isr,        // 92 --
    unused_isr,        // 93 --
    unused_isr,        // 94 --
    unused_isr,        // 95 --
    unused_isr,        // 96 --
    dac0_isr,          // 97 DAC0
    unused_isr,        // 98 --
    tsi0_isr,          // 99 TSI0
    mcg_isr,           // 100 MCG
    lptmr_isr,         // 101 Low Power Timer
    unused_isr,        // 102 --
    porta_isr,         // 103 Pin detect (Port A)
    portb_isr,         // 104 Pin detect (Port B)
    portc_isr,         // 105 Pin detect (Port C)
    portd_isr,         // 106 Pin detect (Port D)
    porte_isr,         // 107 Pin detect (Port E)
    unused_isr,        // 108 --
    unused_isr,        // 109 --
    software_isr,      // 110 Software interrupt
#elif defined(__MK64FX512__)
    dma_ch0_isr,       // 16 DMA channel 0 transfer complete
    dma_ch1_isr,       // 17 DMA channel 1 transfer complete
    dma_ch2_isr,       // 18 DMA channel 2 transfer complete
    dma_ch3_isr,       // 19 DMA channel 3 transfer complete
    dma_ch4_isr,       // 20 DMA channel 4 transfer complete
    dma_ch5_isr,       // 21 DMA channel 5 transfer complete
    dma_ch6_isr,       // 22 DMA channel 6 transfer complete
    dma_ch7_isr,       // 23 DMA channel 7 transfer complete
    dma_ch8_isr,       // 24 DMA channel 8 transfer complete
    dma_ch9_isr,       // 25 DMA channel 9 transfer complete
    dma_ch10_isr,      // 26 DMA channel 10 transfer complete
    dma_ch11_isr,      // 27 DMA channel 11 transfer complete
    dma_ch12_isr,      // 28 DMA channel 12 transfer complete
    dma_ch13_isr,      // 29 DMA channel 13 transfer complete
    dma_ch14_isr,      // 30 DMA channel 14 transfer complete
    dma_ch15_isr,      // 31 DMA channel 15 transfer complete
    dma_error_isr,     // 32 DMA error interrupt channel
    mcm_isr,           // 33 MCM
    flash_cmd_isr,     // 34 Flash Memory Command complete
    flash_error_isr,   // 35 Flash Read collision
    low_voltage_isr,   // 36 Low-voltage detect/warning
    wakeup_isr,        // 37 Low Leakage Wakeup
    watchdog_isr,      // 38 Both EWM and WDOG interrupt
    randnum_isr,       // 39 Random Number Generator
    i2c0_isr,          // 40 I2C0
    i2c1_isr,          // 41 I2C1
    spi0_isr,          // 42 SPI0
    spi1_isr,          // 43 SPI1
    i2s0_tx_isr,       // 44 I2S0 Transmit
    i2s0_rx_isr,       // 45 I2S0 Receive
    unused_isr,        // 46 --
    uart0_status_isr,  // 47 UART0 status
    uart0_error_isr,   // 48 UART0 error
    uart1_status_isr,  // 49 UART1 status
    uart1_error_isr,   // 50 UART1 error
    uart2_status_isr,  // 51 UART2 status
    uart2_error_isr,   // 52 UART2 error
    uart3_status_isr,  // 53 UART3 status
    uart3_error_isr,   // 54 UART3 error
    adc0_isr,          // 55 ADC0
    cmp0_isr,          // 56 CMP0
    cmp1_isr,          // 57 CMP1
    ftm0_isr,          // 58 FTM0
    ftm1_isr,          // 59 FTM1
    ftm2_isr,          // 60 FTM2
    cmt_isr,           // 61 CMT
    rtc_alarm_isr,     // 62 RTC Alarm interrupt
    rtc_seconds_isr,   // 63 RTC Seconds interrupt
    pit0_isr,          // 64 PIT Channel 0
    pit1_isr,          // 65 PIT Channel 1
    pit2_isr,          // 66 PIT Channel 2
    pit3_isr,          // 67 PIT Channel 3
    pdb_isr,           // 68 PDB Programmable Delay Block
    usb_isr,           // 69 USB OTG
    usb_charge_isr,    // 70 USB Charger Detect
    unused_isr,        // 71 --
    dac0_isr,          // 72 DAC0
    mcg_isr,           // 73 MCG
    lptmr_isr,         // 74 Low Power Timer
    porta_isr,         // 75 Pin detect (Port A)
    portb_isr,         // 76 Pin detect (Port B)
    portc_isr,         // 77 Pin detect (Port C)
    portd_isr,         // 78 Pin detect (Port D)
    porte_isr,         // 79 Pin detect (Port E)
    software_isr,      // 80 Software interrupt
    spi2_isr,          // 81 SPI2
    uart4_status_isr,  // 82 UART4 status
    uart4_error_isr,   // 83 UART4 error
    uart5_status_isr,  // 84 UART4 status
    uart5_error_isr,   // 85 UART4 error
    cmp2_isr,          // 86 CMP2
    ftm3_isr,          // 87 FTM3
    dac1_isr,          // 88 DAC1
    adc1_isr,          // 89 ADC1
    i2c2_isr,          // 90 I2C2
    can0_message_isr,  // 91 CAN OR'ed Message buffer (0-15)
    can0_bus_off_isr,  // 92 CAN Bus Off
    can0_error_isr,    // 93 CAN Error
    can0_tx_warn_isr,  // 94 CAN Transmit Warning
    can0_rx_warn_isr,  // 95 CAN Receive Warning
    can0_wakeup_isr,   // 96 CAN Wake Up
    sdhc_isr,          // 97 SDHC
    enet_timer_isr,    // 98 Ethernet IEEE1588 Timers
    enet_tx_isr,       // 99 Ethernet Transmit
    enet_rx_isr,       // 100 Ethernet Receive
    enet_error_isr,    // 101 Ethernet Error
#elif defined(__MK22FX512__)
    dma_ch0_isr,       // 16 DMA channel 0 transfer complete
    dma_ch1_isr,       // 17 DMA channel 1 transfer complete
    dma_ch2_isr,       // 18 DMA channel 2 transfer complete
    dma_ch3_isr,       // 19 DMA channel 3 transfer complete
    dma_ch4_isr,       // 20 DMA channel 4 transfer complete
    dma_ch5_isr,       // 21 DMA channel 5 transfer complete
    dma_ch6_isr,       // 22 DMA channel 6 transfer complete
    dma_ch7_isr,       // 23 DMA channel 7 transfer complete
    dma_ch8_isr,       // 24 DMA channel 8 transfer complete
    dma_ch9_isr,       // 25 DMA channel 9 transfer complete
    dma_ch10_isr,      // 26 DMA channel 10 transfer complete
    dma_ch11_isr,      // 27 DMA channel 11 transfer complete
    dma_ch12_isr,      // 28 DMA channel 12 transfer complete
    dma_ch13_isr,      // 29 DMA channel 13 transfer complete
    dma_ch14_isr,      // 30 DMA channel 14 transfer complete
    dma_ch15_isr,      // 31 DMA channel 15 transfer complete
    dma_error_isr,     // 32 DMA error interrupt channel
    mcm_isr,           // 33 MCM
    flash_cmd_isr,     // 34 Flash Memory Command complete
    flash_error_isr,   // 35 Flash Read collision
    low_voltage_isr,   // 36 Low-voltage detect/warning
    wakeup_isr,        // 37 Low Leakage Wakeup
    watchdog_isr,      // 38 Both EWM and WDOG interrupt
    unused_isr,        // 39 --
    i2c0_isr,          // 40 I2C0
    i2c1_isr,          // 41 I2C1
    spi0_isr,          // 42 SPI0
    spi1_isr,          // 43 SPI1
    i2s0_tx_isr,       // 44 I2S0 Transmit
    i2s0_rx_isr,       // 45 I2S0 Receive
    unused_isr,        // 46 --
    uart0_status_isr,  // 47 UART0 status
    uart0_error_isr,   // 48 UART0 error
    uart1_status_isr,  // 49 UART1 status
    uart1_error_isr,   // 50 UART1 error
    uart2_status_isr,  // 51 UART2 status
    uart2_error_isr,   // 52 UART2 error
    uart3_status_isr,  // 53 UART3 status
    uart3_error_isr,   // 54 UART3 error
    adc0_isr,          // 55 ADC0
    cmp0_isr,          // 56 CMP0
    cmp1_isr,          // 57 CMP1
    ftm0_isr,          // 58 FTM0
    ftm1_isr,          // 59 FTM1
    ftm2_isr,          // 60 FTM2
    cmt_isr,           // 61 CMT
    rtc_alarm_isr,     // 62 RTC Alarm interrupt
    rtc_seconds_isr,   // 63 RTC Seconds interrupt
    pit0_isr,          // 64 PIT Channel 0
    pit1_isr,          // 65 PIT Channel 1
    pit2_isr,          // 66 PIT Channel 2
    pit3_isr,          // 67 PIT Channel 3
    pdb_isr,           // 68 PDB Programmable Delay Block
    usb_isr,           // 69 USB OTG
    usb_charge_isr,    // 70 USB Charger Detect
    unused_isr,        // 71 --
    dac0_isr,          // 72 DAC0
    mcg_isr,           // 73 MCG
    lptmr_isr,         // 74 Low Power Timer
    porta_isr,         // 75 Pin detect (Port A)
    portb_isr,         // 76 Pin detect (Port B)
    portc_isr,         // 77 Pin detect (Port C)
    portd_isr,         // 78 Pin detect (Port D)
    porte_isr,         // 79 Pin detect (Port E)
    software_isr,      // 80 Software interrupt
    unused_isr,        // 81 --
    unused_isr,        // 82 --
    unused_isr,        // 83 --
    unused_isr,        // 84 --
    unused_isr,        // 85 --
    cmp2_isr,          // 86 CMP2
    ftm3_isr,          // 87 FTM3
    unused_isr,        // 88 --
    adc1_isr,          // 89 ADC1
    i2c2_isr,          // 90 I2C2
    can0_message_isr,  // 91 CAN OR'ed Message buffer (0-15)
    can0_bus_off_isr,  // 92 CAN Bus Off
    can0_error_isr,    // 93 CAN Error
    can0_tx_warn_isr,  // 94 CAN Transmit Warning
    can0_rx_warn_isr,  // 95 CAN Receive Warning
    can0_wakeup_isr,   // 96 CAN Wake Up
    sdhc_isr,          // 97 SDHC
#else
#error
#endif
};

__attribute__((section(".flashconfig"), used))
const uint8_t flashconfigbytes[16] = {0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF, /* Backdoor access key */
                                      0xFF,
                                      0xFF,
                                      0xFF,
                                      0xFF /* Program flash protection */,
                                      FSEC,
                                      FOPT,
                                      0xFF /* FEPROT */,
                                      0xFF /* PDPROT */};

#ifdef __clang__
// Clang seems to generate slightly larger code with Os than gcc
__attribute__((optimize("-Os")))
#else
__attribute__((section(".startup"), optimize("-Os")))
#endif
void ResetHandler(void) {
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  __asm__ __volatile__("nop" ::: "memory");
  __asm__ __volatile__("nop" ::: "memory");

  WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;

  // enable clocks to always-used peripherals
#if defined(__MK20DX256__)
  SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2;
  SIM_SCGC5 = 0x00043F82;  // clocks active to all GPIO
  SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 |
              SIM_SCGC6_FTF;
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MK22FX512__)
  SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
  SIM_SCGC5 = 0x00043F82;  // clocks active to all GPIO
  SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 |
              SIM_SCGC6_FTF;
  SCB_CPACR = 0x00F00000;
#else
#error
#endif
  // if the RTC oscillator isn't enabled, get it started early
  if (!(RTC_CR & RTC_CR_OSCE)) {
    RTC_SR = 0;
    RTC_CR = RTC_CR_SC16P | RTC_CR_SC4P | RTC_CR_OSCE;
  }
  // release I/O pins hold, if we woke up from VLLS mode
  if (PMC_REGSC & PMC_REGSC_ACKISO) {
    PMC_REGSC |= PMC_REGSC_ACKISO;
  }

  // since this is a write once register, make it visible to all F_CPU's
  // so we can into other sleep modes in the future at any speed
  SMC_PMPROT = SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;

  {
    uint32_t *src, *dest, *end;
    __asm__("ldr %0, =__data_flash_start__" :"=r"(src));
    __asm__("ldr %0, =__data_ram_start__" :"=r"(dest));
    __asm__("ldr %0, =__data_ram_end__" :"=r"(end));
    while (dest < end) {
      *dest++ = *src++;
    }
  }
  {
    uint32_t *src, *dest, *end;
    __asm__("ldr %0, =__sram_l_flash_start__" :"=r"(src));
    __asm__("ldr %0, =__data_sram_l_start__" :"=r"(dest));
    __asm__("ldr %0, =__data_sram_l_end__" :"=r"(end));
    while (dest < end) {
      *dest++ = *src++;
    }
  }
  {
    uint32_t *dest, *end;
    __asm__("ldr %0, =__bss_ram_start__" :"=r"(dest));
    __asm__("ldr %0, =__bss_ram_end__" :"=r"(end));
    while (dest < end) {
      *dest++ = 0;
    }
  }

  // default all interrupts to medium priority level
  for (int i = 0; i < NVIC_NUM_INTERRUPTS + 16; i++) {
    _VectorsRam[i] = _VectorsFlash[i];
  }
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++) {
    NVIC_SET_PRIORITY(i, 128);
  }
  SCB_VTOR = (uint32_t)_VectorsRam;  // use vector table in RAM

  // hardware always starts in FEI mode
  //  C1[CLKS] bits are written to 00
  //  C1[IREFS] bit is written to 1
  //  C6[PLLS] bit is written to 0
  // MCG_SC[FCDIV] defaults to divide by two for internal ref clock
  // I tried changing MSG_SC to divide by 1, it didn't work for me
  // enable capacitors for crystal
  OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
  // enable osc, 8-32 MHz range, low power mode
  MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
  // switch to crystal as clock source, FLL input = 16 MHz / 512
  MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
  // wait for crystal oscillator to begin
  while ((MCG_S & MCG_S_OSCINIT0) == 0) {
  }
  // wait for FLL to use oscillator
  while ((MCG_S & MCG_S_IREFST) != 0) {
  }
  // wait for MCGOUT to use oscillator
  while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) {
  }

  // now in FBE mode
  //  C1[CLKS] bits are written to 10
  //  C1[IREFS] bit is written to 0
  //  C1[FRDIV] must be written to divide xtal to 31.25-39 kHz
  //  C6[PLLS] bit is written to 0
  //  C2[LP] is written to 0
  // if we need faster than the crystal, turn on the PLL
#if F_CPU == 72000000
  MCG_C5 =
      MCG_C5_PRDIV0(5);  // config PLL input for 16 MHz Crystal / 6 = 2.667 Hz
#else
  MCG_C5 = MCG_C5_PRDIV0(3);               // config PLL input for 16 MHz Crystal / 4 = 4 MHz
#endif
#if F_CPU == 120000000
  MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(6);  // config PLL for 120 MHz output
#elif F_CPU == 72000000
  MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(3);  // config PLL for 72 MHz output
#else
#error "Unsupported F_CPU"
#endif

  // wait for PLL to start using xtal as its input
  while (!(MCG_S & MCG_S_PLLST)) {
  }
  // wait for PLL to lock
  while (!(MCG_S & MCG_S_LOCK0)) {
  }
  // now we're in PBE mode
  // now program the clock dividers
#if F_CPU == 120000000
  // config divisors: 120 MHz core, 60 MHz bus, 40 MHz FlexBus, 24 MHz flash,
  // USB = 120 * 2 / 5 = 48 MHz
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) |
                SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(4);
  SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(4) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 72000000
  // config divisors: 72 MHz core, 36 MHz bus, 36 MHz FlexBus, 24 MHz flash,
  // USB = 72 * 2 / 3 = 48 MHz
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) |
                SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(2);
  SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC;
#else
#error "Unsupported F_CPU"
#endif

  // switch to PLL as clock source, FLL input = 16 MHz / 512
  MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
  // wait for PLL clock to be used
  while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3)) {
  }
  // now we're in PEE mode
  // USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0
  SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL |
              SIM_SOPT2_CLKOUTSEL(6);

  // initialize the SysTick counter
  SYST_RVR = (F_CPU / 1000) - 1;
  SYST_CVR = 0;
  SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
  SCB_SHPR3 = 0x20200000;  // Systick = priority 32

  __enable_irq();

  __libc_init_array();

  main();
  while (1) {
  }
}

char *__brkval = (char *)__heap_start__;

void *_sbrk(int incr) {
  char *prev;

  prev = __brkval;
  if (incr != 0) {
    if (prev + incr >= (char *)__heap_end__) {
      errno = ENOMEM;
      return (void *)-1;
    }
    __brkval = prev + incr;
  }
  return prev;
}

__attribute__((weak)) int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

__attribute__((weak)) int _close(int fd) {
  (void)fd;
  return -1;
}

#include <sys/stat.h>

__attribute__((weak)) int _fstat(int fd, struct stat *st) {
  (void)fd;
  st->st_mode = S_IFCHR;
  return 0;
}

__attribute__((weak)) int _isatty(int fd) {
  (void)fd;
  return 1;
}

__attribute__((weak)) int _lseek(int fd, long long offset, int whence) {
  (void)fd;
  (void)offset;
  (void)whence;
  return -1;
}

__attribute__((weak)) void _exit(int status) {
  (void)status;
  while (1) {
  }
}

__attribute__((weak)) void _kill(pid_t pid, int signal) {
  (void)pid;
  (void)signal;
  while (1) {
  }
}

__attribute__((weak)) int _getpid() { return -1; }

__attribute__((weak)) void __cxa_pure_virtual() {
  while (1) {
  }
}

__attribute__((weak)) int __cxa_guard_acquire(char *g) { return !(*g); }

__attribute__((weak)) void __cxa_guard_release(char *g) { *g = 1; }
