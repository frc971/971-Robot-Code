/**
 ******************************************************************************
 * @file    stm32g4xx_ll_adc.c
 * @author  MCD Application Team
 * @brief   ADC LL module driver
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"

#include "stm32g4xx_ll_bus.h"

#ifdef USE_FULL_ASSERT
#include "stm32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/** @addtogroup STM32G4xx_LL_Driver
 * @{
 */

#if defined(ADC1) || defined(ADC2) || defined(ADC3) || defined(ADC4) || \
    defined(ADC5)

/** @addtogroup ADC_LL ADC
 * @{
 */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup ADC_LL_Private_Constants
 * @{
 */

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC peripheral HW delays are defined in ADC LL driver driver,   */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */
/* Note: ADC timeout values are defined here in CPU cycles to be independent  */
/*       of device clock setting.                                             */
/*       In user application, ADC timeout values should be defined with       */
/*       temporal values, in function of device clock settings.               */
/*       Highest ratio CPU clock frequency vs ADC clock frequency:            */
/*        - ADC clock from synchronous clock with AHB prescaler 512,          */
/*          ADC prescaler 4.                                                  */
/*           Ratio max = 512 *4 = 2048                                        */
/*        - ADC clock from asynchronous clock (PLLP) with prescaler 256.      */
/*          Highest CPU clock PLL (PLLR).                                     */
/*           Ratio max = PLLRmax /PPLPmin * 256 = (VCO/2) / (VCO/31) * 256    */
/*                     = 3968                                                 */
/* Unit: CPU cycles.                                                          */
#define ADC_CLOCK_RATIO_VS_CPU_HIGHEST (3968UL)
#define ADC_TIMEOUT_DISABLE_CPU_CYCLES (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)
#define ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES \
  (ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1UL)

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/

/** @addtogroup ADC_LL_Private_Macros
 * @{
 */

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* common to several ADC instances.                                           */
#define IS_LL_ADC_COMMON_CLOCK(__CLOCK__)          \
  (((__CLOCK__) == LL_ADC_CLOCK_SYNC_PCLK_DIV1) || \
   ((__CLOCK__) == LL_ADC_CLOCK_SYNC_PCLK_DIV2) || \
   ((__CLOCK__) == LL_ADC_CLOCK_SYNC_PCLK_DIV4) || \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV1) ||     \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV2) ||     \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV4) ||     \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV6) ||     \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV8) ||     \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV10) ||    \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV12) ||    \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV16) ||    \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV32) ||    \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV64) ||    \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV128) ||   \
   ((__CLOCK__) == LL_ADC_CLOCK_ASYNC_DIV256))

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC instance.                                                              */
#define IS_LL_ADC_RESOLUTION(__RESOLUTION__)      \
  (((__RESOLUTION__) == LL_ADC_RESOLUTION_12B) || \
   ((__RESOLUTION__) == LL_ADC_RESOLUTION_10B) || \
   ((__RESOLUTION__) == LL_ADC_RESOLUTION_8B) ||  \
   ((__RESOLUTION__) == LL_ADC_RESOLUTION_6B))

#define IS_LL_ADC_DATA_ALIGN(__DATA_ALIGN__)        \
  (((__DATA_ALIGN__) == LL_ADC_DATA_ALIGN_RIGHT) || \
   ((__DATA_ALIGN__) == LL_ADC_DATA_ALIGN_LEFT))

#define IS_LL_ADC_LOW_POWER(__LOW_POWER__)     \
  (((__LOW_POWER__) == LL_ADC_LP_MODE_NONE) || \
   ((__LOW_POWER__) == LL_ADC_LP_AUTOWAIT))

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group regular                                                          */
#if defined(STM32G474xx) || defined(STM32G484xx)
#define IS_LL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__) \
  (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE) ||                \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM7_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH1) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG1) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG3) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG5) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG6) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG7) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG8) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG9) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG10) ||         \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH2) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH3) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11))) ||     \
   ((((__ADC_INSTANCE__) == ADC3) || ((__ADC_INSTANCE__) == ADC4) ||     \
     ((__ADC_INSTANCE__) == ADC5)) &&                                    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH3) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE2) ||        \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG2) ||        \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_HRTIM_TRG4))))
#elif defined(STM32G473xx) || defined(STM32G483xx)
#define IS_LL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__) \
  (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE) ||                \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM7_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH1) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH2) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH3) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11))) ||     \
   ((((__ADC_INSTANCE__) == ADC3) || ((__ADC_INSTANCE__) == ADC4) ||     \
     ((__ADC_INSTANCE__) == ADC5)) &&                                    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH3) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE2))))
#elif defined(STM32G471xx)
#define IS_LL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__) \
  (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE) ||                \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM7_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11))) ||     \
   (((__ADC_INSTANCE__) == ADC3) &&                                      \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH3) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE2))))
#elif defined(STM32G411xB) || defined(STM32G411xC) || defined(STM32G414xx) || \
    defined(STM32GBK1CB) || defined(STM32G431xx) || defined(STM32G441xx)
#define IS_LL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__) \
  (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE) ||                \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM7_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM_OUT) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11))
#elif defined(STM32G491xx) || defined(STM32G4A1xx)
#define IS_LL_ADC_REG_TRIG_SOURCE(__ADC_INSTANCE__, __REG_TRIG_SOURCE__) \
  (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_SOFTWARE) ||                \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH3) ||            \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM6_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM7_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM15_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO) ||          \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH1) ||           \
   ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM1_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH2) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH4) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH2) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM20_CH3) ||         \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE11))) ||     \
   (((__ADC_INSTANCE__) == ADC3) &&                                      \
    (((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM2_CH3) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM3_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM4_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_TIM8_CH1) ||          \
     ((__REG_TRIG_SOURCE__) == LL_ADC_REG_TRIG_EXT_EXTI_LINE2))))
#endif /* STM32G4xx */

#define IS_LL_ADC_REG_CONTINUOUS_MODE(__REG_CONTINUOUS_MODE__) \
  (((__REG_CONTINUOUS_MODE__) == LL_ADC_REG_CONV_SINGLE) ||    \
   ((__REG_CONTINUOUS_MODE__) == LL_ADC_REG_CONV_CONTINUOUS))

#define IS_LL_ADC_REG_DMA_TRANSFER(__REG_DMA_TRANSFER__)          \
  (((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_NONE) ||    \
   ((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_LIMITED) || \
   ((__REG_DMA_TRANSFER__) == LL_ADC_REG_DMA_TRANSFER_UNLIMITED))

#define IS_LL_ADC_REG_OVR_DATA_BEHAVIOR(__REG_OVR_DATA_BEHAVIOR__)   \
  (((__REG_OVR_DATA_BEHAVIOR__) == LL_ADC_REG_OVR_DATA_PRESERVED) || \
   ((__REG_OVR_DATA_BEHAVIOR__) == LL_ADC_REG_OVR_DATA_OVERWRITTEN))

#define IS_LL_ADC_REG_SEQ_SCAN_LENGTH(__REG_SEQ_SCAN_LENGTH__)          \
  (((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_DISABLE) ||        \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS) ||  \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS) || \
   ((__REG_SEQ_SCAN_LENGTH__) == LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS))

#define IS_LL_ADC_REG_SEQ_SCAN_DISCONT_MODE(__REG_SEQ_DISCONT_MODE__) \
  (((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_DISABLE) ||  \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_1RANK) ||    \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_2RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_3RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_4RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_5RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_6RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_7RANKS) ||   \
   ((__REG_SEQ_DISCONT_MODE__) == LL_ADC_REG_SEQ_DISCONT_8RANKS))

/* Check of parameters for configuration of ADC hierarchical scope:           */
/* ADC group injected                                                         */
#if defined(STM32G474xx) || defined(STM32G484xx)
#define IS_LL_ADC_INJ_TRIG_SOURCE(__ADC_INSTANCE__, __INJ_TRIG_SOURCE__) \
  (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE) ||                \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM7_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG4) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG5) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG6) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG7) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG8) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG9) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG10) ||         \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM16_CH1) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH4) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15))) ||     \
   ((((__ADC_INSTANCE__) == ADC3) || ((__ADC_INSTANCE__) == ADC4) ||     \
     ((__ADC_INSTANCE__) == ADC5)) &&                                    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH2) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH2) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG1) ||        \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_HRTIM_TRG3) ||        \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE3))))
#elif defined(STM32G473xx) || defined(STM32G483xx)
#define IS_LL_ADC_INJ_TRIG_SOURCE(__ADC_INSTANCE__, __INJ_TRIG_SOURCE__) \
  (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE) ||                \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM7_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM16_CH1) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH4) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15))) ||     \
   ((((__ADC_INSTANCE__) == ADC3) || ((__ADC_INSTANCE__) == ADC4) ||     \
     ((__ADC_INSTANCE__) == ADC5)) &&                                    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH2) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH2) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE3))))
#elif defined(STM32G471xx)
#define IS_LL_ADC_INJ_TRIG_SOURCE(__ADC_INSTANCE__, __INJ_TRIG_SOURCE__) \
  (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE) ||                \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM7_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM16_CH1) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15))) ||     \
   ((((__ADC_INSTANCE__) == ADC3)) &&                                    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH2) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE3))))
#elif defined(STM32G411xB) || defined(STM32G411xC) || defined(STM32G414xx) || \
    defined(STM32GBK1CB) || defined(STM32G431xx) || defined(STM32G441xx)
#define IS_LL_ADC_INJ_TRIG_SOURCE(__ADC_INSTANCE__, __INJ_TRIG_SOURCE__) \
  (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE) ||                \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM7_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM16_CH1) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM_OUT) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15))
#elif defined(STM32G491xx) || defined(STM32G4A1xx)
#define IS_LL_ADC_INJ_TRIG_SOURCE(__ADC_INSTANCE__, __INJ_TRIG_SOURCE__) \
  (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_SOFTWARE) ||                \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM6_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM7_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO) ||           \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH4) ||            \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM15_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO) ||          \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_TRGO2) ||         \
   ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_LPTIM_OUT) ||           \
   ((((__ADC_INSTANCE__) == ADC1) || ((__ADC_INSTANCE__) == ADC2)) &&    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM2_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH1) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM3_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM16_CH1) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH4) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE15))) ||     \
   ((((__ADC_INSTANCE__) == ADC3)) &&                                    \
    (((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM1_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH3) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM4_CH4) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM8_CH2) ||          \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_TIM20_CH2) ||         \
     ((__INJ_TRIG_SOURCE__) == LL_ADC_INJ_TRIG_EXT_EXTI_LINE3))))
#endif /* STM32G4xx */

#define IS_LL_ADC_INJ_TRIG_EXT_EDGE(__INJ_TRIG_EXT_EDGE__)     \
  (((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_RISING) ||  \
   ((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_FALLING) || \
   ((__INJ_TRIG_EXT_EDGE__) == LL_ADC_INJ_TRIG_EXT_RISINGFALLING))

#define IS_LL_ADC_INJ_TRIG_AUTO(__INJ_TRIG_AUTO__)         \
  (((__INJ_TRIG_AUTO__) == LL_ADC_INJ_TRIG_INDEPENDENT) || \
   ((__INJ_TRIG_AUTO__) == LL_ADC_INJ_TRIG_FROM_GRP_REGULAR))

#define IS_LL_ADC_INJ_SEQ_SCAN_LENGTH(__INJ_SEQ_SCAN_LENGTH__)         \
  (((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_DISABLE) ||       \
   ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS) || \
   ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS) || \
   ((__INJ_SEQ_SCAN_LENGTH__) == LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS))

#define IS_LL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(__INJ_SEQ_DISCONT_MODE__) \
  (((__INJ_SEQ_DISCONT_MODE__) == LL_ADC_INJ_SEQ_DISCONT_DISABLE) ||  \
   ((__INJ_SEQ_DISCONT_MODE__) == LL_ADC_INJ_SEQ_DISCONT_1RANK))

#if defined(ADC_MULTIMODE_SUPPORT)
/* Check of parameters for configuration of ADC hierarchical scope:           */
/* multimode.                                                                 */
#define IS_LL_ADC_MULTI_MODE(__MULTI_MODE__)                  \
  (((__MULTI_MODE__) == LL_ADC_MULTI_INDEPENDENT) ||          \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_REG_SIMULT) ||      \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_REG_INTERL) ||      \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_INJ_SIMULT) ||      \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_INJ_ALTERN) ||      \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_REG_SIM_INJ_SIM) || \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_REG_SIM_INJ_ALT) || \
   ((__MULTI_MODE__) == LL_ADC_MULTI_DUAL_REG_INT_INJ_SIM))

#define IS_LL_ADC_MULTI_DMA_TRANSFER(__MULTI_DMA_TRANSFER__)             \
  (((__MULTI_DMA_TRANSFER__) == LL_ADC_MULTI_REG_DMA_EACH_ADC) ||        \
   ((__MULTI_DMA_TRANSFER__) == LL_ADC_MULTI_REG_DMA_LIMIT_RES12_10B) || \
   ((__MULTI_DMA_TRANSFER__) == LL_ADC_MULTI_REG_DMA_LIMIT_RES8_6B) ||   \
   ((__MULTI_DMA_TRANSFER__) == LL_ADC_MULTI_REG_DMA_UNLMT_RES12_10B) || \
   ((__MULTI_DMA_TRANSFER__) == LL_ADC_MULTI_REG_DMA_UNLMT_RES8_6B))

#define IS_LL_ADC_MULTI_TWOSMP_DELAY(__MULTI_TWOSMP_DELAY__)           \
  (((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE) ||   \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_2CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_3CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_4CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_6CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_7CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_8CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_9CYCLES) ||  \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_10CYCLES) || \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_11CYCLES) || \
   ((__MULTI_TWOSMP_DELAY__) == LL_ADC_MULTI_TWOSMP_DELAY_12CYCLES))

#define IS_LL_ADC_MULTI_MASTER_SLAVE(__MULTI_MASTER_SLAVE__) \
  (((__MULTI_MASTER_SLAVE__) == LL_ADC_MULTI_MASTER) ||      \
   ((__MULTI_MASTER_SLAVE__) == LL_ADC_MULTI_SLAVE) ||       \
   ((__MULTI_MASTER_SLAVE__) == LL_ADC_MULTI_MASTER_SLAVE))

#endif /* ADC_MULTIMODE_SUPPORT */
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_LL_Exported_Functions
 * @{
 */

/** @addtogroup ADC_LL_EF_Init
 * @{
 */

/**
 * @brief  De-initialize registers of all ADC instances belonging to
 *         the same ADC common instance to their default reset values.
 * @note   This function is performing a hard reset, using high level
 *         clock source RCC ADC reset.
 *         Caution: On this STM32 series, if several ADC instances are available
 *         on the selected device, RCC ADC reset will reset
 *         all ADC instances belonging to the common ADC instance.
 *         To de-initialize only 1 ADC instance, use
 *         function @ref LL_ADC_DeInit().
 * @param  ADCxy_COMMON ADC common instance
 *         (can be set directly from CMSIS definition or by using helper macro
 * @ref __LL_ADC_COMMON_INSTANCE() )
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC common registers are de-initialized
 *          - ERROR: not applicable
 */
ErrorStatus LL_ADC_CommonDeInit(const ADC_Common_TypeDef *ADCxy_COMMON) {
  /* Check the parameters */
  assert_param(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));

  if (ADCxy_COMMON == ADC12_COMMON) {
    /* Force reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC12);

    /* Release reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC12);
  }
#if defined(ADC345_COMMON)
  else {
    /* Force reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC345);

    /* Release reset of ADC clock (core clock) */
    LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC345);
  }
#endif /* ADC345_COMMON */

  return SUCCESS;
}

/**
 * @brief  Initialize some features of ADC common parameters
 *         (all ADC instances belonging to the same ADC common instance)
 *         and multimode (for devices with several ADC instances available).
 * @note   The setting of ADC common parameters is conditioned to
 *         ADC instances state:
 *         All ADC instances belonging to the same ADC common instance
 *         must be disabled.
 * @param  ADCxy_COMMON ADC common instance
 *         (can be set directly from CMSIS definition or by using helper macro
 * @ref __LL_ADC_COMMON_INSTANCE() )
 * @param  pADC_CommonInitStruct Pointer to a @ref LL_ADC_CommonInitTypeDef
 * structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC common registers are initialized
 *          - ERROR: ADC common registers are not initialized
 */
ErrorStatus LL_ADC_CommonInit(
    ADC_Common_TypeDef *ADCxy_COMMON,
    const LL_ADC_CommonInitTypeDef *pADC_CommonInitStruct) {
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_COMMON_INSTANCE(ADCxy_COMMON));
  assert_param(IS_LL_ADC_COMMON_CLOCK(pADC_CommonInitStruct->CommonClock));

#if defined(ADC_MULTIMODE_SUPPORT)
  assert_param(IS_LL_ADC_MULTI_MODE(pADC_CommonInitStruct->Multimode));
  if (pADC_CommonInitStruct->Multimode != LL_ADC_MULTI_INDEPENDENT) {
    assert_param(
        IS_LL_ADC_MULTI_DMA_TRANSFER(pADC_CommonInitStruct->MultiDMATransfer));
    assert_param(IS_LL_ADC_MULTI_TWOSMP_DELAY(
        pADC_CommonInitStruct->MultiTwoSamplingDelay));
  }
#endif /* ADC_MULTIMODE_SUPPORT */

  /* Note: Hardware constraint (refer to description of functions             */
  /*       "LL_ADC_SetCommonXXX()" and "LL_ADC_SetMultiXXX()"):               */
  /*       On this STM32 series, setting of these features is conditioned to  */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  if (__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(ADCxy_COMMON) == 0UL) {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - common to several ADC                                               */
    /*    (all ADC instances belonging to the same ADC common instance)       */
    /*    - Set ADC clock (conversion clock)                                  */
    /*  - multimode (if several ADC instances available on the                */
    /*    selected device)                                                    */
    /*    - Set ADC multimode configuration                                   */
    /*    - Set ADC multimode DMA transfer                                    */
    /*    - Set ADC multimode: delay between 2 sampling phases                */
#if defined(ADC_MULTIMODE_SUPPORT)
    if (pADC_CommonInitStruct->Multimode != LL_ADC_MULTI_INDEPENDENT) {
      MODIFY_REG(ADCxy_COMMON->CCR,
                 ADC_CCR_CKMODE | ADC_CCR_PRESC | ADC_CCR_DUAL | ADC_CCR_MDMA |
                     ADC_CCR_DELAY,
                 pADC_CommonInitStruct->CommonClock |
                     pADC_CommonInitStruct->Multimode |
                     pADC_CommonInitStruct->MultiDMATransfer |
                     pADC_CommonInitStruct->MultiTwoSamplingDelay);
    } else {
      MODIFY_REG(ADCxy_COMMON->CCR,
                 ADC_CCR_CKMODE | ADC_CCR_PRESC | ADC_CCR_DUAL | ADC_CCR_MDMA |
                     ADC_CCR_DELAY,
                 pADC_CommonInitStruct->CommonClock | LL_ADC_MULTI_INDEPENDENT);
    }
#else
    LL_ADC_SetCommonClock(ADCxy_COMMON, pADC_CommonInitStruct->CommonClock);
#endif /* ADC_MULTIMODE_SUPPORT */
  } else {
    /* Initialization error: One or several ADC instances belonging to        */
    /* the same ADC common instance are not disabled.                         */
    status = ERROR;
  }

  return status;
}

/**
 * @brief  Set each @ref LL_ADC_CommonInitTypeDef field to default value.
 * @param  pADC_CommonInitStruct Pointer to a @ref LL_ADC_CommonInitTypeDef
 * structure whose fields will be set to default values.
 * @retval None
 */
void LL_ADC_CommonStructInit(LL_ADC_CommonInitTypeDef *pADC_CommonInitStruct) {
  /* Set pADC_CommonInitStruct fields to default values */
  /* Set fields of ADC common */
  /* (all ADC instances belonging to the same ADC common instance) */
  pADC_CommonInitStruct->CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;

#if defined(ADC_MULTIMODE_SUPPORT)
  /* Set fields of ADC multimode */
  pADC_CommonInitStruct->Multimode = LL_ADC_MULTI_INDEPENDENT;
  pADC_CommonInitStruct->MultiDMATransfer = LL_ADC_MULTI_REG_DMA_EACH_ADC;
  pADC_CommonInitStruct->MultiTwoSamplingDelay =
      LL_ADC_MULTI_TWOSMP_DELAY_1CYCLE;
#endif /* ADC_MULTIMODE_SUPPORT */
}

/**
 * @brief  De-initialize registers of the selected ADC instance
 *         to their default reset values.
 * @note   To reset all ADC instances quickly (perform a hard reset),
 *         use function @ref LL_ADC_CommonDeInit().
 * @note   If this functions returns error status, it means that ADC instance
 *         is in an unknown state.
 *         In this case, perform a hard reset using high level
 *         clock source RCC ADC reset.
 *         Caution: On this STM32 series, if several ADC instances are available
 *         on the selected device, RCC ADC reset will reset
 *         all ADC instances belonging to the common ADC instance.
 *         Refer to function @ref LL_ADC_CommonDeInit().
 * @param  ADCx ADC instance
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC registers are de-initialized
 *          - ERROR: ADC registers are not de-initialized
 */
ErrorStatus LL_ADC_DeInit(ADC_TypeDef *ADCx) {
  ErrorStatus status = SUCCESS;

  __IO uint32_t timeout_cpu_cycles = 0UL;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));

  /* Disable ADC instance if not already disabled.                            */
  if (LL_ADC_IsEnabled(ADCx) == 1UL) {
    /* Stop potential ADC conversion on going on ADC group regular.           */
    if (LL_ADC_REG_IsConversionOngoing(ADCx) != 0UL) {
      if (LL_ADC_REG_IsStopConversionOngoing(ADCx) == 0UL) {
        LL_ADC_REG_StopConversion(ADCx);
      }
    }

    /* Stop potential ADC conversion on going on ADC group injected.          */
    if (LL_ADC_INJ_IsConversionOngoing(ADCx) != 0UL) {
      if (LL_ADC_INJ_IsStopConversionOngoing(ADCx) == 0UL) {
        LL_ADC_INJ_StopConversion(ADCx);
      }
    }

    /* Wait for ADC conversions are effectively stopped                       */
    timeout_cpu_cycles = ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES;
    while ((LL_ADC_REG_IsStopConversionOngoing(ADCx) |
            LL_ADC_INJ_IsStopConversionOngoing(ADCx)) == 1UL) {
      timeout_cpu_cycles--;
      if (timeout_cpu_cycles == 0UL) {
        /* Time-out error */
        status = ERROR;
        break;
      }
    }

    /* Flush group injected contexts queue (register JSQR):                   */
    /* Note: Bit JQM must be set to empty the contexts queue (otherwise       */
    /*       contexts queue is maintained with the last active context).      */
    LL_ADC_INJ_SetQueueMode(ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);

    /* Disable the ADC instance */
    LL_ADC_Disable(ADCx);

    /* Wait for ADC instance is effectively disabled */
    timeout_cpu_cycles = ADC_TIMEOUT_DISABLE_CPU_CYCLES;
    while (LL_ADC_IsDisableOngoing(ADCx) == 1UL) {
      timeout_cpu_cycles--;
      if (timeout_cpu_cycles == 0UL) {
        /* Time-out error */
        status = ERROR;
        break;
      }
    }
  }

  /* Check whether ADC state is compliant with expected state */
  if (READ_BIT(ADCx->CR, (ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART |
                          ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)) ==
      0UL) {
    /* ========== Reset ADC registers ========== */
    /* Reset register IER */
    CLEAR_BIT(ADCx->IER, (LL_ADC_IT_ADRDY | LL_ADC_IT_EOC | LL_ADC_IT_EOS |
                          LL_ADC_IT_OVR | LL_ADC_IT_EOSMP | LL_ADC_IT_JEOC |
                          LL_ADC_IT_JEOS | LL_ADC_IT_JQOVF | LL_ADC_IT_AWD1 |
                          LL_ADC_IT_AWD2 | LL_ADC_IT_AWD3));

    /* Reset register ISR */
    SET_BIT(ADCx->ISR,
            (LL_ADC_FLAG_ADRDY | LL_ADC_FLAG_EOC | LL_ADC_FLAG_EOS |
             LL_ADC_FLAG_OVR | LL_ADC_FLAG_EOSMP | LL_ADC_FLAG_JEOC |
             LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JQOVF | LL_ADC_FLAG_AWD1 |
             LL_ADC_FLAG_AWD2 | LL_ADC_FLAG_AWD3));

    /* Reset register CR */
    /*  - Bits ADC_CR_JADSTP, ADC_CR_ADSTP, ADC_CR_JADSTART, ADC_CR_ADSTART,  */
    /*    ADC_CR_ADCAL, ADC_CR_ADDIS, ADC_CR_ADEN are in                      */
    /*    access mode "read-set": no direct reset applicable.                 */
    /*  - Reset Calibration mode to default setting (single ended).           */
    /*  - Disable ADC internal voltage regulator.                             */
    /*  - Enable ADC deep power down.                                         */
    /*    Note: ADC internal voltage regulator disable and ADC deep power     */
    /*          down enable are conditioned to ADC state disabled:            */
    /*          already done above.                                           */
    CLEAR_BIT(ADCx->CR, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
    SET_BIT(ADCx->CR, ADC_CR_DEEPPWD);

    /* Reset register CFGR */
    MODIFY_REG(
        ADCx->CFGR,
        (ADC_CFGR_AWD1CH | ADC_CFGR_JAUTO | ADC_CFGR_JAWD1EN | ADC_CFGR_AWD1EN |
         ADC_CFGR_AWD1SGL | ADC_CFGR_JQM | ADC_CFGR_JDISCEN | ADC_CFGR_DISCNUM |
         ADC_CFGR_DISCEN | ADC_CFGR_AUTDLY | ADC_CFGR_CONT | ADC_CFGR_OVRMOD |
         ADC_CFGR_EXTEN | ADC_CFGR_EXTSEL | ADC_CFGR_ALIGN | ADC_CFGR_RES |
         ADC_CFGR_DMACFG | ADC_CFGR_DMAEN),
        ADC_CFGR_JQDIS);

    /* Reset register CFGR2 */
    CLEAR_BIT(
        ADCx->CFGR2,
        (ADC_CFGR2_ROVSM | ADC_CFGR2_TROVS | ADC_CFGR2_OVSS | ADC_CFGR2_SWTRIG |
         ADC_CFGR2_BULB | ADC_CFGR2_SMPTRIG | ADC_CFGR2_GCOMP | ADC_CFGR2_OVSR |
         ADC_CFGR2_JOVSE | ADC_CFGR2_ROVSE));

    /* Reset register SMPR1 */
    CLEAR_BIT(ADCx->SMPR1, (ADC_SMPR1_SMP9 | ADC_SMPR1_SMP8 | ADC_SMPR1_SMP7 |
                            ADC_SMPR1_SMP6 | ADC_SMPR1_SMP5 | ADC_SMPR1_SMP4 |
                            ADC_SMPR1_SMP3 | ADC_SMPR1_SMP2 | ADC_SMPR1_SMP1));

    /* Reset register SMPR2 */
    CLEAR_BIT(ADCx->SMPR2,
              (ADC_SMPR2_SMP18 | ADC_SMPR2_SMP17 | ADC_SMPR2_SMP16 |
               ADC_SMPR2_SMP15 | ADC_SMPR2_SMP14 | ADC_SMPR2_SMP13 |
               ADC_SMPR2_SMP12 | ADC_SMPR2_SMP11 | ADC_SMPR2_SMP10));

    /* Reset register TR1 */
    MODIFY_REG(ADCx->TR1, ADC_TR1_AWDFILT | ADC_TR1_HT1 | ADC_TR1_LT1,
               ADC_TR1_HT1);

    /* Reset register TR2 */
    MODIFY_REG(ADCx->TR2, ADC_TR2_HT2 | ADC_TR2_LT2, ADC_TR2_HT2);

    /* Reset register TR3 */
    MODIFY_REG(ADCx->TR3, ADC_TR3_HT3 | ADC_TR3_LT3, ADC_TR3_HT3);

    /* Reset register SQR1 */
    CLEAR_BIT(ADCx->SQR1, (ADC_SQR1_SQ4 | ADC_SQR1_SQ3 | ADC_SQR1_SQ2 |
                           ADC_SQR1_SQ1 | ADC_SQR1_L));

    /* Reset register SQR2 */
    CLEAR_BIT(ADCx->SQR2, (ADC_SQR2_SQ9 | ADC_SQR2_SQ8 | ADC_SQR2_SQ7 |
                           ADC_SQR2_SQ6 | ADC_SQR2_SQ5));

    /* Reset register SQR3 */
    CLEAR_BIT(ADCx->SQR3, (ADC_SQR3_SQ14 | ADC_SQR3_SQ13 | ADC_SQR3_SQ12 |
                           ADC_SQR3_SQ11 | ADC_SQR3_SQ10));

    /* Reset register SQR4 */
    CLEAR_BIT(ADCx->SQR4, ADC_SQR4_SQ16 | ADC_SQR4_SQ15);

    /* Reset register JSQR */
    CLEAR_BIT(ADCx->JSQR,
              (ADC_JSQR_JL | ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN |
               ADC_JSQR_JSQ4 | ADC_JSQR_JSQ3 | ADC_JSQR_JSQ2 | ADC_JSQR_JSQ1));

    /* Reset register DR */
    /* Note: bits in access mode read only, no direct reset applicable */

    /* Reset register OFR1 */
    CLEAR_BIT(ADCx->OFR1, ADC_OFR1_OFFSET1_EN | ADC_OFR1_OFFSET1_CH |
                              ADC_OFR1_OFFSET1 | ADC_OFR1_SATEN |
                              ADC_OFR1_OFFSETPOS);
    /* Reset register OFR2 */
    CLEAR_BIT(ADCx->OFR2, ADC_OFR2_OFFSET2_EN | ADC_OFR2_OFFSET2_CH |
                              ADC_OFR2_OFFSET2 | ADC_OFR2_SATEN |
                              ADC_OFR2_OFFSETPOS);
    /* Reset register OFR3 */
    CLEAR_BIT(ADCx->OFR3, ADC_OFR3_OFFSET3_EN | ADC_OFR3_OFFSET3_CH |
                              ADC_OFR3_OFFSET3 | ADC_OFR3_SATEN |
                              ADC_OFR3_OFFSETPOS);
    /* Reset register OFR4 */
    CLEAR_BIT(ADCx->OFR4, ADC_OFR4_OFFSET4_EN | ADC_OFR4_OFFSET4_CH |
                              ADC_OFR4_OFFSET4 | ADC_OFR4_SATEN |
                              ADC_OFR4_OFFSETPOS);

    /* Reset registers JDR1, JDR2, JDR3, JDR4 */
    /* Note: bits in access mode read only, no direct reset applicable */

    /* Reset register AWD2CR */
    CLEAR_BIT(ADCx->AWD2CR, ADC_AWD2CR_AWD2CH);

    /* Reset register AWD3CR */
    CLEAR_BIT(ADCx->AWD3CR, ADC_AWD3CR_AWD3CH);

    /* Reset register DIFSEL */
    CLEAR_BIT(ADCx->DIFSEL, ADC_DIFSEL_DIFSEL);

    /* Reset register CALFACT */
    CLEAR_BIT(ADCx->CALFACT, ADC_CALFACT_CALFACT_D | ADC_CALFACT_CALFACT_S);

    /* Reset register GCOMP */
    CLEAR_BIT(ADCx->GCOMP, ADC_GCOMP_GCOMPCOEFF);
  } else {
    /* ADC instance is in an unknown state */
    /* Need to performing a hard reset of ADC instance, using high level      */
    /* clock source RCC ADC reset.                                            */
    /* Caution: On this STM32 series, if several ADC instances are available  */
    /*          on the selected device, RCC ADC reset will reset              */
    /*          all ADC instances belonging to the common ADC instance.       */
    /* Caution: On this STM32 series, if several ADC instances are available  */
    /*          on the selected device, RCC ADC reset will reset              */
    /*          all ADC instances belonging to the common ADC instance.       */
    status = ERROR;
  }

  return status;
}

/**
 * @brief  Initialize some features of ADC instance.
 * @note   These parameters have an impact on ADC scope: ADC instance.
 *         Affects both group regular and group injected (availability
 *         of ADC group injected depends on STM32 series).
 *         Refer to corresponding unitary functions into
 *         @ref ADC_LL_EF_Configuration_ADC_Instance .
 * @note   The setting of these parameters by function @ref LL_ADC_Init()
 *         is conditioned to ADC state:
 *         ADC instance must be disabled.
 *         This condition is applied to all ADC features, for efficiency
 *         and compatibility over all STM32 series. However, the different
 *         features can be set under different ADC state conditions
 *         (setting possible with ADC enabled without conversion on going,
 *         ADC enabled with conversion on going, ...)
 *         Each feature can be updated afterwards with a unitary function
 *         and potentially with ADC in a different state than disabled,
 *         refer to description of each function for setting
 *         conditioned to ADC state.
 * @note   After using this function, some other features must be configured
 *         using LL unitary functions.
 *         The minimum configuration remaining to be done is:
 *          - Set ADC group regular or group injected sequencer:
 *            map channel on the selected sequencer rank.
 *            Refer to function @ref LL_ADC_REG_SetSequencerRanks().
 *          - Set ADC channel sampling time
 *            Refer to function LL_ADC_SetChannelSamplingTime();
 * @param  ADCx ADC instance
 * @param  pADC_InitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC registers are initialized
 *          - ERROR: ADC registers are not initialized
 */
ErrorStatus LL_ADC_Init(ADC_TypeDef *ADCx,
                        const LL_ADC_InitTypeDef *pADC_InitStruct) {
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));

  assert_param(IS_LL_ADC_RESOLUTION(pADC_InitStruct->Resolution));
  assert_param(IS_LL_ADC_DATA_ALIGN(pADC_InitStruct->DataAlignment));
  assert_param(IS_LL_ADC_LOW_POWER(pADC_InitStruct->LowPowerMode));

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL) {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC instance                                                        */
    /*    - Set ADC data resolution                                           */
    /*    - Set ADC conversion data alignment                                 */
    /*    - Set ADC low power mode                                            */
    MODIFY_REG(ADCx->CFGR, ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_AUTDLY,
               pADC_InitStruct->Resolution | pADC_InitStruct->DataAlignment |
                   pADC_InitStruct->LowPowerMode);

  } else {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }

  return status;
}

/**
 * @brief  Set each @ref LL_ADC_InitTypeDef field to default value.
 * @param  pADC_InitStruct Pointer to a @ref LL_ADC_InitTypeDef structure
 *                        whose fields will be set to default values.
 * @retval None
 */
void LL_ADC_StructInit(LL_ADC_InitTypeDef *pADC_InitStruct) {
  /* Set pADC_InitStruct fields to default values */
  /* Set fields of ADC instance */
  pADC_InitStruct->Resolution = LL_ADC_RESOLUTION_12B;
  pADC_InitStruct->DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  pADC_InitStruct->LowPowerMode = LL_ADC_LP_MODE_NONE;
}

/**
 * @brief  Initialize some features of ADC group regular.
 * @note   These parameters have an impact on ADC scope: ADC group regular.
 *         Refer to corresponding unitary functions into
 *         @ref ADC_LL_EF_Configuration_ADC_Group_Regular
 *         (functions with prefix "REG").
 * @note   The setting of these parameters by function @ref LL_ADC_Init()
 *         is conditioned to ADC state:
 *         ADC instance must be disabled.
 *         This condition is applied to all ADC features, for efficiency
 *         and compatibility over all STM32 series. However, the different
 *         features can be set under different ADC state conditions
 *         (setting possible with ADC enabled without conversion on going,
 *         ADC enabled with conversion on going, ...)
 *         Each feature can be updated afterwards with a unitary function
 *         and potentially with ADC in a different state than disabled,
 *         refer to description of each function for setting
 *         conditioned to ADC state.
 * @note   After using this function, other features must be configured
 *         using LL unitary functions.
 *         The minimum configuration remaining to be done is:
 *          - Set ADC group regular or group injected sequencer:
 *            map channel on the selected sequencer rank.
 *            Refer to function @ref LL_ADC_REG_SetSequencerRanks().
 *          - Set ADC channel sampling time
 *            Refer to function LL_ADC_SetChannelSamplingTime();
 * @param  ADCx ADC instance
 * @param  pADC_RegInitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC registers are initialized
 *          - ERROR: ADC registers are not initialized
 */
ErrorStatus LL_ADC_REG_Init(ADC_TypeDef *ADCx,
                            const LL_ADC_REG_InitTypeDef *pADC_RegInitStruct) {
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));
  assert_param(
      IS_LL_ADC_REG_TRIG_SOURCE(ADCx, pADC_RegInitStruct->TriggerSource));
  assert_param(
      IS_LL_ADC_REG_SEQ_SCAN_LENGTH(pADC_RegInitStruct->SequencerLength));
  if (pADC_RegInitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE) {
    assert_param(IS_LL_ADC_REG_SEQ_SCAN_DISCONT_MODE(
        pADC_RegInitStruct->SequencerDiscont));

    /* ADC group regular continuous mode and discontinuous mode */
    /* can not be enabled simultenaeously */
    assert_param(
        (pADC_RegInitStruct->ContinuousMode == LL_ADC_REG_CONV_SINGLE) ||
        (pADC_RegInitStruct->SequencerDiscont ==
         LL_ADC_REG_SEQ_DISCONT_DISABLE));
  }
  assert_param(
      IS_LL_ADC_REG_CONTINUOUS_MODE(pADC_RegInitStruct->ContinuousMode));
  assert_param(IS_LL_ADC_REG_DMA_TRANSFER(pADC_RegInitStruct->DMATransfer));
  assert_param(IS_LL_ADC_REG_OVR_DATA_BEHAVIOR(pADC_RegInitStruct->Overrun));

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL) {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group regular                                                   */
    /*    - Set ADC group regular trigger source                              */
    /*    - Set ADC group regular sequencer length                            */
    /*    - Set ADC group regular sequencer discontinuous mode                */
    /*    - Set ADC group regular continuous mode                             */
    /*    - Set ADC group regular conversion data transfer: no transfer or    */
    /*      transfer by DMA, and DMA requests mode                            */
    /*    - Set ADC group regular overrun behavior                            */
    /* Note: On this STM32 series, ADC trigger edge is set to value 0x0 by    */
    /*       setting of trigger source to SW start.                           */
    if (pADC_RegInitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE) {
      MODIFY_REG(ADCx->CFGR,
                 ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN | ADC_CFGR_DISCEN |
                     ADC_CFGR_DISCNUM | ADC_CFGR_CONT | ADC_CFGR_DMAEN |
                     ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD,
                 pADC_RegInitStruct->TriggerSource |
                     pADC_RegInitStruct->SequencerDiscont |
                     pADC_RegInitStruct->ContinuousMode |
                     pADC_RegInitStruct->DMATransfer |
                     pADC_RegInitStruct->Overrun);
    } else {
      MODIFY_REG(
          ADCx->CFGR,
          ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN | ADC_CFGR_DISCEN |
              ADC_CFGR_DISCNUM | ADC_CFGR_CONT | ADC_CFGR_DMAEN |
              ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD,
          pADC_RegInitStruct->TriggerSource | LL_ADC_REG_SEQ_DISCONT_DISABLE |
              pADC_RegInitStruct->ContinuousMode |
              pADC_RegInitStruct->DMATransfer | pADC_RegInitStruct->Overrun);
    }

    /* Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADCx, pADC_RegInitStruct->SequencerLength);
  } else {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
 * @brief  Set each @ref LL_ADC_REG_InitTypeDef field to default value.
 * @param  pADC_RegInitStruct Pointer to a @ref LL_ADC_REG_InitTypeDef structure
 *                            whose fields will be set to default values.
 * @retval None
 */
void LL_ADC_REG_StructInit(LL_ADC_REG_InitTypeDef *pADC_RegInitStruct) {
  /* Set pADC_RegInitStruct fields to default values */
  /* Set fields of ADC group regular */
  /* Note: On this STM32 series, ADC trigger edge is set to value 0x0 by      */
  /*       setting of trigger source to SW start.                             */
  pADC_RegInitStruct->TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  pADC_RegInitStruct->SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  pADC_RegInitStruct->SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  pADC_RegInitStruct->ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  pADC_RegInitStruct->DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  pADC_RegInitStruct->Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
}

/**
 * @brief  Initialize some features of ADC group injected.
 * @note   These parameters have an impact on ADC scope: ADC group injected.
 *         Refer to corresponding unitary functions into
 *         @ref ADC_LL_EF_Configuration_ADC_Group_Regular
 *         (functions with prefix "INJ").
 * @note   The setting of these parameters by function @ref LL_ADC_Init()
 *         is conditioned to ADC state:
 *         ADC instance must be disabled.
 *         This condition is applied to all ADC features, for efficiency
 *         and compatibility over all STM32 series. However, the different
 *         features can be set under different ADC state conditions
 *         (setting possible with ADC enabled without conversion on going,
 *         ADC enabled with conversion on going, ...)
 *         Each feature can be updated afterwards with a unitary function
 *         and potentially with ADC in a different state than disabled,
 *         refer to description of each function for setting
 *         conditioned to ADC state.
 * @note   After using this function, other features must be configured
 *         using LL unitary functions.
 *         The minimum configuration remaining to be done is:
 *          - Set ADC group injected sequencer:
 *            map channel on the selected sequencer rank.
 *            Refer to function @ref LL_ADC_INJ_SetSequencerRanks().
 *          - Set ADC channel sampling time
 *            Refer to function LL_ADC_SetChannelSamplingTime();
 * @note   Caution if feature ADC group injected contexts queue is enabled
 *         (refer to with function @ref LL_ADC_INJ_SetQueueMode() ):
 *         using successively several times this function will appear as
 *         having no effect.
 *         To set several features of ADC group injected, use
 *         function @ref LL_ADC_INJ_ConfigQueueContext().
 * @param  ADCx ADC instance
 * @param  pADC_InjInitStruct Pointer to a @ref LL_ADC_INJ_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: ADC registers are initialized
 *          - ERROR: ADC registers are not initialized
 */
ErrorStatus LL_ADC_INJ_Init(ADC_TypeDef *ADCx,
                            const LL_ADC_INJ_InitTypeDef *pADC_InjInitStruct) {
  ErrorStatus status = SUCCESS;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(ADCx));
  assert_param(
      IS_LL_ADC_INJ_TRIG_SOURCE(ADCx, pADC_InjInitStruct->TriggerSource));
  assert_param(
      IS_LL_ADC_INJ_SEQ_SCAN_LENGTH(pADC_InjInitStruct->SequencerLength));
  if (pADC_InjInitStruct->SequencerLength != LL_ADC_INJ_SEQ_SCAN_DISABLE) {
    assert_param(IS_LL_ADC_INJ_SEQ_SCAN_DISCONT_MODE(
        pADC_InjInitStruct->SequencerDiscont));
  }
  assert_param(IS_LL_ADC_INJ_TRIG_AUTO(pADC_InjInitStruct->TrigAuto));

  /* Note: Hardware constraint (refer to description of this function):       */
  /*       ADC instance must be disabled.                                     */
  if (LL_ADC_IsEnabled(ADCx) == 0UL) {
    /* Configuration of ADC hierarchical scope:                               */
    /*  - ADC group injected                                                  */
    /*    - Set ADC group injected trigger source                             */
    /*    - Set ADC group injected sequencer length                           */
    /*    - Set ADC group injected sequencer discontinuous mode               */
    /*    - Set ADC group injected conversion trigger: independent or         */
    /*      from ADC group regular                                            */
    /* Note: On this STM32 series, ADC trigger edge is set to value 0x0 by    */
    /*       setting of trigger source to SW start.                           */
    if (pADC_InjInitStruct->SequencerLength != LL_ADC_REG_SEQ_SCAN_DISABLE) {
      MODIFY_REG(
          ADCx->CFGR, ADC_CFGR_JDISCEN | ADC_CFGR_JAUTO,
          pADC_InjInitStruct->SequencerDiscont | pADC_InjInitStruct->TrigAuto);
    } else {
      MODIFY_REG(ADCx->CFGR, ADC_CFGR_JDISCEN | ADC_CFGR_JAUTO,
                 LL_ADC_REG_SEQ_DISCONT_DISABLE | pADC_InjInitStruct->TrigAuto);
    }

    MODIFY_REG(ADCx->JSQR, ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN | ADC_JSQR_JL,
               pADC_InjInitStruct->TriggerSource |
                   pADC_InjInitStruct->SequencerLength);
  } else {
    /* Initialization error: ADC instance is not disabled. */
    status = ERROR;
  }
  return status;
}

/**
 * @brief  Set each @ref LL_ADC_INJ_InitTypeDef field to default value.
 * @param  pADC_InjInitStruct Pointer to a @ref LL_ADC_INJ_InitTypeDef structure
 *                            whose fields will be set to default values.
 * @retval None
 */
void LL_ADC_INJ_StructInit(LL_ADC_INJ_InitTypeDef *pADC_InjInitStruct) {
  /* Set pADC_InjInitStruct fields to default values */
  /* Set fields of ADC group injected */
  pADC_InjInitStruct->TriggerSource = LL_ADC_INJ_TRIG_SOFTWARE;
  pADC_InjInitStruct->SequencerLength = LL_ADC_INJ_SEQ_SCAN_DISABLE;
  pADC_InjInitStruct->SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  pADC_InjInitStruct->TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* ADC1 || ADC2 || ADC3 || ADC4 || ADC5 */

/**
 * @}
 */

#endif /* USE_FULL_LL_DRIVER */
