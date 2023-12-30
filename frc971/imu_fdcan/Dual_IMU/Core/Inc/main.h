/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Different representations of the same four byte value. Simplifies conversion.
union FourBytes {
  float decimal;
  uint32_t four_bytes;
  uint16_t two_bytes[2];
  uint8_t byte[4];
};
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define LED1_GRN_Pin GPIO_PIN_0
#define LED1_GRN_GPIO_Port GPIOC
#define LED1_RED_Pin GPIO_PIN_1
#define LED1_RED_GPIO_Port GPIOC
#define LED2_GRN_Pin GPIO_PIN_2
#define LED2_GRN_GPIO_Port GPIOC
#define LED2_RED_Pin GPIO_PIN_3
#define LED2_RED_GPIO_Port GPIOC
#define INT_TDK_Pin GPIO_PIN_0
#define INT_TDK_GPIO_Port GPIOA
#define TDK_EN_Pin GPIO_PIN_1
#define TDK_EN_GPIO_Port GPIOA
#define FSYNC_TDK_ST_Pin GPIO_PIN_2
#define FSYNC_TDK_ST_GPIO_Port GPIOA
#define RESET_UNO_Pin GPIO_PIN_3
#define RESET_UNO_GPIO_Port GPIOA
#define CS_TDK_ST_Pin GPIO_PIN_4
#define CS_TDK_ST_GPIO_Port GPIOA
#define SCK_TDK_ST_Pin GPIO_PIN_5
#define SCK_TDK_ST_GPIO_Port GPIOA
#define MISO_TDK_Pin GPIO_PIN_6
#define MISO_TDK_GPIO_Port GPIOA
#define MOSI_TDK_ST_Pin GPIO_PIN_7
#define MOSI_TDK_ST_GPIO_Port GPIOA
#define T_VCP_TX_Pin GPIO_PIN_4
#define T_VCP_TX_GPIO_Port GPIOC
#define T_VCP_RX_Pin GPIO_PIN_5
#define T_VCP_RX_GPIO_Port GPIOC
#define CS_UNO_Pin GPIO_PIN_12
#define CS_UNO_GPIO_Port GPIOB
#define SCK_UNO_Pin GPIO_PIN_13
#define SCK_UNO_GPIO_Port GPIOB
#define MISO_UNO_Pin GPIO_PIN_14
#define MISO_UNO_GPIO_Port GPIOB
#define MOSI_UNO_Pin GPIO_PIN_15
#define MOSI_UNO_GPIO_Port GPIOB
#define RESET_DUE_Pin GPIO_PIN_6
#define RESET_DUE_GPIO_Port GPIOC
#define TDK_PWR_EN_Pin GPIO_PIN_9
#define TDK_PWR_EN_GPIO_Port GPIOC
#define VBAT_ADC_Pin GPIO_PIN_8
#define VBAT_ADC_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define CS_DUE_Pin GPIO_PIN_15
#define CS_DUE_GPIO_Port GPIOA
#define SCK_DUE_Pin GPIO_PIN_10
#define SCK_DUE_GPIO_Port GPIOC
#define MISO_DUE_Pin GPIO_PIN_11
#define MISO_DUE_GPIO_Port GPIOC
#define MOSI_DUE_Pin GPIO_PIN_12
#define MOSI_DUE_GPIO_Port GPIOC
#define FDCAN_RX_Pin GPIO_PIN_5
#define FDCAN_RX_GPIO_Port GPIOB
#define FDCAN_TX_Pin GPIO_PIN_6
#define FDCAN_TX_GPIO_Port GPIOB
#define FDCAN_STBY_Pin GPIO_PIN_7
#define FDCAN_STBY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
