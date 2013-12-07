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
.code 16  

.section .stack
  .align 3

  .equ  Stack_Size, 0x00000400

 .globl  __StackLimit
 __StackLimit:
  .space  Stack_Size
  .size __StackLimit, . - __StackLimit

 .globl  __StackTop
 __StackTop:
  .size __StackTop, . - __StackTop

.section .heap
  .align 3

 .globl  __HeapBase
 __HeapBase:
  .equ  Heap_Size, 0x00000C00
  .if  Heap_Size
    .space  Heap_Size
  .endif
  .size __HeapBase, . - __HeapBase

 .globl  __HeapLimit
 __HeapLimit:
  .size __HeapLimit, . - __HeapLimit

.macro ISR_HANDLER name=
  .section .vectors, "a"
    .word \name
  .section .init, "ax"
   .thumb_func
   .weak \name
   \name:
    1: b 1b /* endless loop */
.endm

.macro ISR_RESERVED
  .section .vectors, "a"
    .word 0
.endm

.section .vectors, "a"
 .global _vectors
 _vectors:
  .long  __StackTop
  .long  Reset_Handler
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

.section .vectors, "a"
 _vectors_end:

.text
 .thumb_func
 .align 2
 .globl  Reset_Handler
 .type  Reset_Handler, %function
 Reset_Handler:

/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */
  ldr  r1, =__etext
  ldr  r2, =__data_start__
  ldr  r3, =__data_end__
  subs  r3, r2
  ble  .LC1
 .LC0:
  subs  r3, #4
  ldr  r0, [r1, r3]
  str  r0, [r2, r3]
  bgt  .LC0
 .LC1:

/*     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
  ldr r1, =__bss_start__
  ldr r2, =__bss_end__
  movs  r0, 0
 .LC2:
  cmp   r1, r2
  itt  lt
  strlt   r0, [r1], #4
  blt  .LC2

  bl  _start
  1: b 1b /* endless loop if it ever returns */

  .pool
  .size Reset_Handler, . - Reset_Handler
