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
typedef union {
  float decimal;
  uint32_t four_bytes;
  uint16_t two_bytes[2];
  uint8_t byte[4];
} FourBytes;

typedef enum { SPI_INIT, SPI_ZERO, SPI_START, SPI_RUN } SpiIn;

typedef enum { SPI_READY, SPI_BUSY } SpiOut;

typedef struct {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
  int state;
  int index;
} DataRawInt16;

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temp;
} DataOutFloat;

typedef struct {
  uint32_t timestamp;
  uint16_t can_counter;
  uint16_t tdk_counter;
  uint16_t uno_counter;
  uint16_t due_counter;
  float tdk_acc_x;
  float tdk_acc_y;
  float tdk_acc_z;
  float tdk_gyro_x;
  float tdk_gyro_y;
  float tdk_gyro_z;
  float murata_acc_x;
  float murata_acc_y;
  float murata_acc_z;
  float murata_gyro_x;
  float murata_gyro_y;
  float murata_gyro_z;
  uint8_t tdk_temp;
  uint8_t uno_temp;
  uint8_t due_temp;
  uint8_t flags;
} CanData;

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
#define PWM_RATE_Pin GPIO_PIN_0
#define PWM_RATE_GPIO_Port GPIOB
#define PWM_HEADING_Pin GPIO_PIN_1
#define PWM_HEADING_GPIO_Port GPIOB
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

#define LSI_CLOCK_FREQ_HZ 32000  // 32 kHz
#define IMU_SAMPLES_PER_MS 3
#define CAN_WATCHDOG_TIMEOUT_US 20000  // 20 ms
#define PWM_WATCHDOG_TIMEOUT_US 20000  // 20 ms
#define SPI_WATCHDOG_TIMEOUT_US 20000  // 20 ms

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
