/* File: startup_ARMCM3.S
 * Purpose: startup file for Cortex-M3 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.4
 * Date: 20 Dezember 2012
 *
 */
/* Copyright (c) 2011 - 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


    .syntax unified
    .arch armv7-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x00000400
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x00000C00
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

#if 0
    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long    WDT_IRQHandler        /*  0:  Watchdog Timer            */
    .long    RTC_IRQHandler        /*  1:  Real Time Clock           */
    .long    TIM0_IRQHandler       /*  2:  Timer0 / Timer1           */
    .long    TIM2_IRQHandler       /*  3:  Timer2 / Timer3           */
    .long    MCIA_IRQHandler       /*  4:  MCIa                      */
    .long    MCIB_IRQHandler       /*  5:  MCIb                      */
    .long    UART0_IRQHandler      /*  6:  UART0 - DUT FPGA          */
    .long    UART1_IRQHandler      /*  7:  UART1 - DUT FPGA          */
    .long    UART2_IRQHandler      /*  8:  UART2 - DUT FPGA          */
    .long    UART4_IRQHandler      /*  9:  UART4 - not connected     */
    .long    AACI_IRQHandler       /* 10: AACI / AC97                */
    .long    CLCD_IRQHandler       /* 11: CLCD Combined Interrupt    */
    .long    ENET_IRQHandler       /* 12: Ethernet                   */
    .long    USBDC_IRQHandler      /* 13: USB Device                 */
    .long    USBHC_IRQHandler      /* 14: USB Host Controller        */
    .long    CHLCD_IRQHandler      /* 15: Character LCD              */
    .long    FLEXRAY_IRQHandler    /* 16: Flexray                    */
    .long    CAN_IRQHandler        /* 17: CAN                        */
    .long    LIN_IRQHandler        /* 18: LIN                        */
    .long    I2C_IRQHandler        /* 19: I2C ADC/DAC                */
    .long    0                     /* 20: Reserved                   */
    .long    0                     /* 21: Reserved                   */
    .long    0                     /* 22: Reserved                   */
    .long    0                     /* 23: Reserved                   */
    .long    0                     /* 24: Reserved                   */
    .long    0                     /* 25: Reserved                   */
    .long    0                     /* 26: Reserved                   */
    .long    0                     /* 27: Reserved                   */
    .long    CPU_CLCD_IRQHandler   /* 28: Reserved - CPU FPGA CLCD   */
    .long    0                     /* 29: Reserved - CPU FPGA        */
    .long    UART3_IRQHandler      /* 30: UART3    - CPU FPGA        */
    .long    SPI_IRQHandler        /* 31: SPI Touchscreen - CPU FPGA */

    .size    __isr_vector, . - __isr_vector
#else
.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler

  .section .vectors, "ax"
  .code 16  
  .global _vectors  

_vectors:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_HANDLER MemManage_Handler 
ISR_HANDLER BusFault_Handler
ISR_HANDLER UsageFault_Handler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_HANDLER DebugMon_Handler
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler 
/* interrupts */
ISR_HANDLER WWDG_IRQHandler
ISR_HANDLER PVD_IRQHandler
ISR_HANDLER TAMP_STAMP_IRQHandler
ISR_HANDLER RTC_WKUP_IRQHandler
ISR_HANDLER FLASH_IRQHandler
ISR_HANDLER RCC_IRQHandler
ISR_HANDLER EXTI0_IRQHandler
ISR_HANDLER EXTI1_IRQHandler
ISR_HANDLER EXTI2_IRQHandler
ISR_HANDLER EXTI3_IRQHandler
ISR_HANDLER EXTI4_IRQHandler
ISR_HANDLER DMA1_Stream0_IRQHandler
ISR_HANDLER DMA1_Stream1_IRQHandler
ISR_HANDLER DMA1_Stream2_IRQHandler
ISR_HANDLER DMA1_Stream3_IRQHandler
ISR_HANDLER DMA1_Stream4_IRQHandler
ISR_HANDLER DMA1_Stream5_IRQHandler
ISR_HANDLER DMA1_Stream6_IRQHandler
ISR_HANDLER ADC_IRQHandler
ISR_HANDLER CAN1_TX_IRQHandler
ISR_HANDLER CAN1_RX0_IRQHandler
ISR_HANDLER CAN1_RX1_IRQHandler
ISR_HANDLER CAN1_SCE_IRQHandler
ISR_HANDLER EXTI9_5_IRQHandler
ISR_HANDLER TIM1_BRK_TIM9_IRQHandler
ISR_HANDLER TIM1_UP_TIM10_IRQHandler
ISR_HANDLER TIM1_TRG_COM_TIM11_IRQHandler
ISR_HANDLER TIM1_CC_IRQHandler
ISR_HANDLER TIM2_IRQHandler
ISR_HANDLER TIM3_IRQHandler
ISR_HANDLER TIM4_IRQHandler
ISR_HANDLER I2C1_EV_IRQHandler
ISR_HANDLER I2C1_ER_IRQHandler
ISR_HANDLER I2C2_EV_IRQHandler
ISR_HANDLER I2C2_ER_IRQHandler
ISR_HANDLER SPI1_IRQHandler
ISR_HANDLER SPI2_IRQHandler
ISR_HANDLER USART1_IRQHandler
ISR_HANDLER USART2_IRQHandler
ISR_HANDLER USART3_IRQHandler
ISR_HANDLER EXTI15_10_IRQHandler
ISR_HANDLER RTC_Alarm_IRQHandler
ISR_HANDLER OTG_FS_WKUP_IRQHandler
ISR_HANDLER TIM8_BRK_TIM12_IRQHandler
ISR_HANDLER TIM8_UP_TIM13_IRQHandler
ISR_HANDLER TIM8_TRG_COM_TIM14_IRQHandler
ISR_HANDLER TIM8_CC_IRQHandler
ISR_HANDLER DMA1_Stream7_IRQHandler
ISR_HANDLER FSMC_IRQHandler
ISR_HANDLER SDIO_IRQHandler
ISR_HANDLER TIM5_IRQHandler
ISR_HANDLER SPI3_IRQHandler
ISR_HANDLER UART4_IRQHandler
ISR_HANDLER UART5_IRQHandler
ISR_HANDLER TIM6_DAC_IRQHandler
ISR_HANDLER TIM7_IRQHandler
ISR_HANDLER DMA2_Stream0_IRQHandler
ISR_HANDLER DMA2_Stream1_IRQHandler
ISR_HANDLER DMA2_Stream2_IRQHandler
ISR_HANDLER DMA2_Stream3_IRQHandler
ISR_HANDLER DMA2_Stream4_IRQHandler
ISR_HANDLER ETH_IRQHandler
ISR_HANDLER ETH_WKUP_IRQHandler
ISR_HANDLER CAN2_TX_IRQHandler
ISR_HANDLER CAN2_RX0_IRQHandler
ISR_HANDLER CAN2_RX1_IRQHandler
ISR_HANDLER CAN2_SCE_IRQHandler
ISR_HANDLER OTG_FS_IRQHandler
ISR_HANDLER DMA2_Stream5_IRQHandler
ISR_HANDLER DMA2_Stream6_IRQHandler
ISR_HANDLER DMA2_Stream7_IRQHandler
ISR_HANDLER USART6_IRQHandler
ISR_HANDLER I2C3_EV_IRQHandler
ISR_HANDLER I2C3_ER_IRQHandler
ISR_HANDLER OTG_HS_EP1_OUT_IRQHandler
ISR_HANDLER OTG_HS_EP1_IN_IRQHandler
ISR_HANDLER OTG_HS_WKUP_IRQHandler
ISR_HANDLER OTG_HS_IRQHandler
ISR_HANDLER DCMI_IRQHandler
ISR_HANDLER CRYP_IRQHandler
ISR_HANDLER HASH_RNG_IRQHandler
  .section .vectors, "ax"
_vectors_end:


  .section .vectors_ram, "ax"
_vectors_ram:
  .space _vectors_end-_vectors, 0
#endif

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:

/*     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2

    bl    SystemInit

  ldr r0, =__vectors_load_start__
  ldr r1, =__vectors_load_end__
  ldr r2, =_vectors_ram
l0:
  cmp r0, r1
  beq l1
  ldr r3, [r0]
  str r3, [r2]
  adds r0, r0, #4
  adds r2, r2, #4
  b l0
l1:

  /* Configure vector table offset register */
  ldr r0, =0xE000ED08
  ldr r1, =_vectors_ram
  str r1, [r0]

#ifndef __START
#define __START _start
#endif
    bl    __START
    .pool
    .size Reset_Handler, . - Reset_Handler
