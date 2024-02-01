/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "murata.h"
#include "tdk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MURATA_ACC_FILTER WRITE_FILTER_46HZ_ACC
#define MURATA_GYRO_FILTER WRITE_FILTER_46HZ_GYRO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc5;

FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

// Redirect printf() to UART
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef tx_header;

static CrossAxisCompMurata
    cac_murata;  // Cross-axis compensation. Details on datasheet page 11
static DataMurataRaw data_murata_raw;
static DataMurata data_murata;
static DataTdkRaw data_tdk_raw;
static DataTdk data_tdk;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_ADC5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

static void EnableLeds(void);
static void EnableSpiTdk(void);
static void ConvertEndianMurata(uint8_t *result, uint32_t original);
static uint8_t MurataCRC8(uint8_t bit_value,
                          uint8_t redundancy);  // For checksum calculation.
                                                // Details on datasheet page 36
static uint8_t GetChecksumMurata(uint32_t data);
static uint32_t MakeSpiMsgMurata(uint8_t address, uint16_t data);
static uint32_t MakeSpiReadMsgMurata(uint8_t address);
static uint32_t TransmitSpiMurata(uint32_t value, GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin, SPI_HandleTypeDef *hspix);
static uint32_t SendSpiUno(uint32_t value);  // Murata UNO
static uint32_t SendSpiDue(uint32_t value);  // Murata DUE
static uint8_t SendSpiTdk(uint8_t address, uint8_t data, bool read);
static bool InitMurata(void);
static void InitTdk(void);
static void ReadDataMurata(void);
static void ReadDataTdk(void);
static void ConvertDataMurata(void);
static void ConvertDataTdk(void);
static void InitCan(FDCAN_TxHeaderTypeDef *tx_header, uint8_t id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_ADC5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  EnableLeds();  // Set LEDs to red
  InitTdk();
  InitMurata();
  InitCan(&tx_header, 0xAA);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ReadDataMurata();
    ConvertDataMurata();
    printf(
        "MUR IMU:\tX ACC: %.8f \tY ACC: %.8f \tZ ACC: %.8f \tX GYRO: %.8f \tY "
        "GYRO: %.8f \tZ GYRO: %.8f \n\r",
        data_murata.acc_x, data_murata.acc_y, data_murata.acc_z,
        data_murata.gyro_x, data_murata.gyro_y, data_murata.gyro_z);

    ReadDataTdk();
    ConvertDataTdk();
    printf(
        "TDK IMU:\t\tX ACC: %.8f \tY ACC: %.8f \tZ ACC: %.8f \tX GYRO: %.8f "
        "\tY GYRO: %.8f \tZ GYRO: %.8f \n\r",
        data_tdk.acc_x, data_tdk.acc_y, data_tdk.acc_z, data_tdk.gyro_x,
        data_tdk.gyro_y, data_tdk.gyro_z);

    uint8_t tx_data[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data) !=
        HAL_OK) {
      Error_Handler();
    }

    HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC5_Init(void) {
  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
   */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {
  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 24;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 3;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 24;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {
  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {
  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2563;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {
  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | LED1_GRN_Pin |
                        LED1_RED_Pin | LED2_GRN_Pin | LED2_RED_Pin |
                        RESET_DUE_Pin | GPIO_PIN_7 | GPIO_PIN_8,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TDK_EN_GPIO_Port, TDK_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,
                    FSYNC_TDK_ST_Pin | RESET_UNO_Pin | CS_TDK_ST_Pin |
                        GPIO_PIN_9 | GPIO_PIN_10 | CS_DUE_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | CS_UNO_Pin |
                        GPIO_PIN_3 | GPIO_PIN_4 | FDCAN_STBY_Pin | GPIO_PIN_9,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TDK_PWR_EN_GPIO_Port, TDK_PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 LED1_GRN_Pin
                           LED1_RED_Pin LED2_GRN_Pin LED2_RED_Pin RESET_DUE_Pin
                           PC7 PC8 TDK_PWR_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | LED1_GRN_Pin |
                        LED1_RED_Pin | LED2_GRN_Pin | LED2_RED_Pin |
                        RESET_DUE_Pin | GPIO_PIN_7 | GPIO_PIN_8 |
                        TDK_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_TDK_Pin */
  GPIO_InitStruct.Pin = INT_TDK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_TDK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TDK_EN_Pin FSYNC_TDK_ST_Pin RESET_UNO_Pin
     CS_TDK_ST_Pin PA9 PA10 CS_DUE_Pin */
  GPIO_InitStruct.Pin = TDK_EN_Pin | FSYNC_TDK_ST_Pin | RESET_UNO_Pin |
                        CS_TDK_ST_Pin | GPIO_PIN_9 | GPIO_PIN_10 | CS_DUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 CS_UNO_Pin
                           PB3 PB4 FDCAN_STBY_Pin PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | CS_UNO_Pin |
                        GPIO_PIN_3 | GPIO_PIN_4 | FDCAN_STBY_Pin | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void EnableLeds(void) {
  HAL_GPIO_WritePin(GPIOC, LED1_GRN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, LED1_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, LED2_GRN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, LED2_RED_Pin, GPIO_PIN_RESET);
}

static void EnableSpiTdk(void) {
  // TDK_PWR_EN starts high, must be set low before reading
  HAL_GPIO_WritePin(TDK_PWR_EN_GPIO_Port, TDK_PWR_EN_Pin, GPIO_PIN_RESET);
  // TDK_EN starts high, must be set low after a delay from TDK_PWR_EN
  HAL_Delay(1000);
  HAL_GPIO_WritePin(TDK_EN_GPIO_Port, TDK_EN_Pin, GPIO_PIN_RESET);
}

static void ConvertEndianMurata(uint8_t *result, uint32_t original) {
  // Reorder bytes to and from Murata's convention. Must be done for all SPI
  // data immediately before sending to and after reading from Murata.
  // o3 o2 o1 o0 --> o2 o3 o0 o1 as r3 r2 r1 r0
  result[0] = (original >> 16) & 0xFF;
  result[1] = (original >> 24) & 0xFF;
  result[2] = original & 0xFF;
  result[3] = (original >> 8) & 0xFF;
}

static uint8_t MurataCRC8(uint8_t bit_value, uint8_t redundancy) {
  // For checksum calculation. See datasheet page 36
  uint8_t temp;
  temp = (uint8_t)(redundancy & 0x80);
  if (bit_value == 0x01) {
    temp ^= 0x80;
  }
  redundancy <<= 1;
  if (temp > 0) {
    redundancy ^= 0x1D;
  }
  return redundancy;
}

static uint8_t GetChecksumMurata(uint32_t data) {
  // Cyclic redundancy check. See datasheet page 36
  uint8_t bit_index;
  uint8_t bit_value;
  uint8_t redundancy;

  redundancy = 0xFF;
  for (bit_index = 31; bit_index > 7; bit_index--) {
    bit_value = (uint8_t)((data >> bit_index) & 0x01);
    redundancy = MurataCRC8(bit_value, redundancy);
  }
  redundancy = (uint8_t)~redundancy;
  return redundancy;
}

static uint32_t MakeSpiMsgMurata(uint8_t address, uint16_t data) {
  // Constructs SPI read/write frame
  // Details on datasheet page 32-34
  uint32_t message = (uint32_t)((((address << 2) | 0x01) << 24) | data << 8);
  uint8_t redundancy = GetChecksumMurata(message);
  return (uint32_t)(message | redundancy);
}

static uint32_t MakeSpiReadMsgMurata(uint8_t address) {
  // Constructs SPI read frame
  return MakeSpiMsgMurata(address, 0x0000);
}

static uint32_t TransmitSpiMurata(uint32_t value, GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin, SPI_HandleTypeDef *hspix) {
  union FourBytes rx_data_raw;
  union FourBytes rx_data;
  union FourBytes tx_data;
  ConvertEndianMurata(tx_data.byte, value);
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(hspix, tx_data.byte, rx_data_raw.byte,
                          sizeof(tx_data.byte) / 2, 100);
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  ConvertEndianMurata(rx_data.byte, rx_data_raw.four_bytes);
  return rx_data.four_bytes;
}

static uint32_t SendSpiUno(uint32_t value) {
  return TransmitSpiMurata(value, CS_UNO_GPIO_Port, CS_UNO_Pin, &hspi2);
}

static uint32_t SendSpiDue(uint32_t value) {
  return TransmitSpiMurata(value, CS_DUE_GPIO_Port, CS_DUE_Pin, &hspi3);
}

static uint8_t SendSpiTdk(uint8_t address, uint8_t data, bool read) {
  uint8_t msb = (read) ? 0x80 : 0x00;
  uint8_t spi_tx[2] = {data, address | msb};
  uint8_t spi_rx[2];
  HAL_GPIO_WritePin(CS_TDK_ST_GPIO_Port, CS_TDK_ST_Pin,
                    GPIO_PIN_RESET);  // CS low at start of transmission
  HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, sizeof(spi_tx) / 2, 100);
  HAL_GPIO_WritePin(CS_TDK_ST_GPIO_Port, CS_TDK_ST_Pin,
                    GPIO_PIN_SET);  // CS high at end of transmission
  return spi_rx[0];
}

static uint8_t ReadSpiTdk(uint8_t address) {
  return SendSpiTdk(address, 0x00, true);
}

static bool InitMurata(void) {
  int num_attempts = 0;
  uint32_t response_due = 0;
  uint32_t response_uno = 0;
  bool due_ok = false;
  bool uno_ok = false;

  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin,
                    GPIO_PIN_RESET);  // Reset UNO
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);

  HAL_Delay(25);

  SendSpiDue(WRITE_MODE_ASM_010);
  SendSpiDue(READ_MODE);
  SendSpiDue(WRITE_MODE_ASM_001);
  SendSpiDue(READ_MODE);
  SendSpiDue(WRITE_MODE_ASM_100);
  SendSpiDue(READ_MODE);
  uint32_t resp = SendSpiDue(READ_MODE);

  if ((GET_SPI_DATA_UINT16(resp) & 0x7) == 7) {
    // Test mode for reading cross-axis terms. Details on datasheet page 14
    SendSpiDue(WRITE_SEL_BANK_5);
    SendSpiDue(MakeSpiReadMsgMurata(ACC_DC1_ADDRESS));  // cxx_cxy address

    uint16_t cxx_cxy = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC9_ADDRESS)));  // cxz_cyx address
    uint16_t cxz_cyx = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC10_ADDRESS)));  // cyy_cyz address
    uint16_t cyy_cyz = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC11_ADDRESS)));  // czx_czy address
    uint16_t czx_czy = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC12_ADDRESS)));  // czz_bxx address
    uint16_t czz_bxx = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC13_ADDRESS)));  // bxy_bxz address
    uint16_t bxy_bxz = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_DC14_ADDRESS)));  // byx_byy address
    uint16_t byx_byy = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_MD1_ADDRESS)));  // byz_bzx address
    uint16_t byz_bzx = GET_SPI_DATA_UINT16(
        SendSpiDue(MakeSpiReadMsgMurata(ACC_MD2_ADDRESS)));  // bzy_bzz address
    uint16_t bzy_bzz =
        GET_SPI_DATA_UINT16(SendSpiDue(MakeSpiReadMsgMurata(ACC_MD2_ADDRESS)));

    cac_murata.cxx = CONV_UINT16_UINT8_L(cxx_cxy) / 4096.0f + 1;
    cac_murata.cxy = CONV_UINT16_UINT8_H(cxx_cxy) / 4096.0f;
    cac_murata.cxz = CONV_UINT16_UINT8_L(cxz_cyx) / 4096.0f;
    cac_murata.cyx = CONV_UINT16_UINT8_H(cxz_cyx) / 4096.0f;
    cac_murata.cyy = CONV_UINT16_UINT8_L(cyy_cyz) / 4096.0f + 1;
    cac_murata.cyz = CONV_UINT16_UINT8_H(cyy_cyz) / 4096.0f;
    cac_murata.czx = CONV_UINT16_UINT8_L(czx_czy) / 4096.0f;
    cac_murata.czy = CONV_UINT16_UINT8_H(czx_czy) / 4096.0f;
    cac_murata.czz = CONV_UINT16_UINT8_L(czz_bxx) / 4096.0f + 1;
    cac_murata.bxx = CONV_UINT16_UINT8_H(czz_bxx) / 4096.0f + 1;
    cac_murata.bxy = CONV_UINT16_UINT8_L(bxy_bxz) / 4096.0f;
    cac_murata.bxz = CONV_UINT16_UINT8_H(bxy_bxz) / 4096.0f;
    cac_murata.byx = CONV_UINT16_UINT8_L(byx_byy) / 4096.0f;
    cac_murata.byy = CONV_UINT16_UINT8_H(byx_byy) / 4096.0f + 1;
    cac_murata.byz = CONV_UINT16_UINT8_L(byz_bzx) / 4096.0f;
    cac_murata.bzx = CONV_UINT16_UINT8_H(byz_bzx) / 4096.0f;
    cac_murata.bzy = CONV_UINT16_UINT8_L(bzy_bzz) / 4096.0f;
    cac_murata.bzz = CONV_UINT16_UINT8_H(bzy_bzz) / 4096.0f + 1;
  } else {
    return false;
  }

  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin,
                    GPIO_PIN_RESET);  // Reset UNO
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);

  HAL_Delay(25);

  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiUno(WRITE_OP_MODE_NORMAL);
  HAL_Delay(70);

  SendSpiUno(MURATA_GYRO_FILTER);
  SendSpiUno(MURATA_ACC_FILTER);

  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);
  HAL_Delay(25);

  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiDue(WRITE_OP_MODE_NORMAL);
  HAL_Delay(1);

  SendSpiDue(MURATA_GYRO_FILTER);

  for (num_attempts = 0; num_attempts < 2; num_attempts++) {
    HAL_Delay(405);

    SendSpiDue(WRITE_EOI_BIT);
    SendSpiUno(WRITE_EOI_BIT);

    /* UNO */
    SendSpiUno(READ_SUMMARY_STATUS);
    SendSpiUno(READ_SUMMARY_STATUS);
    HAL_Delay(3);
    response_uno = SendSpiUno(READ_SUMMARY_STATUS);
    uno_ok = !CHECK_RS_ERROR(response_uno);

    /* DUE */
    SendSpiDue(READ_SUMMARY_STATUS);
    SendSpiDue(READ_SUMMARY_STATUS);
    HAL_Delay(3);
    response_due = SendSpiDue(READ_SUMMARY_STATUS);
    due_ok = !CHECK_RS_ERROR(response_due);

    if ((due_ok == false || uno_ok == false) && (num_attempts == 0)) {
      HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin,
                        GPIO_PIN_RESET);  // Reset UNO
      HAL_Delay(1);
      HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                        GPIO_PIN_RESET);  // Reset DUE
      HAL_Delay(1);
      HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);
      HAL_Delay(25);

      SendSpiDue(WRITE_OP_MODE_NORMAL);
      SendSpiDue(WRITE_OP_MODE_NORMAL);
      SendSpiUno(WRITE_OP_MODE_NORMAL);
      HAL_Delay(50);

      SendSpiUno(MURATA_GYRO_FILTER);
      SendSpiUno(MURATA_ACC_FILTER);
      SendSpiDue(MURATA_GYRO_FILTER);
      HAL_Delay(45);
    } else {
      break;
    }
  }

  if (!due_ok || !uno_ok) {
    return false;
  }

  return true;
}

// Todo (Zach): separate the steps of reading data and realigning axis
static void ReadDataMurata(void) {
  SendSpiUno(READ_ACC_Z);
  data_murata_raw.acc_z = -GET_SPI_DATA_INT16(SendSpiUno(READ_ACC_X));
  data_murata_raw.acc_y = GET_SPI_DATA_INT16(SendSpiUno(READ_ACC_Y));
  data_murata_raw.acc_x = GET_SPI_DATA_INT16(SendSpiUno(READ_GYRO_X));
  data_murata_raw.gyro_y = -GET_SPI_DATA_INT16(SendSpiUno(READ_TEMP));
  data_murata_raw.temp_uno = GET_SPI_DATA_INT16(SendSpiUno(READ_ACC_Z));

  SendSpiDue(READ_GYRO_Z);
  data_murata_raw.gyro_z = GET_SPI_DATA_INT16(SendSpiDue(READ_GYRO_Y));
  data_murata_raw.gyro_x = -GET_SPI_DATA_INT16(SendSpiDue(READ_TEMP));
  data_murata_raw.temp_due = GET_SPI_DATA_INT16(SendSpiDue(READ_GYRO_Z));
}

static void ConvertDataMurata(void) {
  data_murata.acc_x = MURATA_CONV_ACC(data_murata_raw.acc_x);
  data_murata.acc_y = MURATA_CONV_ACC(data_murata_raw.acc_y);
  data_murata.acc_z = MURATA_CONV_ACC(data_murata_raw.acc_z);
  data_murata.gyro_x = MURATA_CONV_GYRO(data_murata_raw.gyro_x);
  data_murata.gyro_y = MURATA_CONV_GYRO(data_murata_raw.gyro_y);
  data_murata.gyro_z = MURATA_CONV_GYRO(data_murata_raw.gyro_z);
  data_murata.temp_uno = MURATA_CONV_TEMP(data_murata_raw.temp_uno);
  data_murata.temp_due = MURATA_CONV_TEMP(data_murata_raw.temp_due);

  data_murata.acc_x = (cac_murata.bxx * data_murata.acc_x) +
                      (cac_murata.bxy * data_murata.acc_y) +
                      (cac_murata.bxz * data_murata.acc_z);
  data_murata.acc_y = (cac_murata.byx * data_murata.acc_x) +
                      (cac_murata.byy * data_murata.acc_y) +
                      (cac_murata.byz * data_murata.acc_z);
  data_murata.acc_z = (cac_murata.bzx * data_murata.acc_x) +
                      (cac_murata.bzy * data_murata.acc_y) +
                      (cac_murata.bzz * data_murata.acc_z);

  data_murata.gyro_x = (cac_murata.cxx * data_murata.gyro_x) +
                       (cac_murata.cxy * data_murata.gyro_y) +
                       (cac_murata.cxz * data_murata.gyro_z);
  data_murata.gyro_y = (cac_murata.cyx * data_murata.gyro_x) +
                       (cac_murata.cyy * data_murata.gyro_y) +
                       (cac_murata.cyz * data_murata.gyro_z);
  data_murata.gyro_z = (cac_murata.czx * data_murata.gyro_x) +
                       (cac_murata.czy * data_murata.gyro_y) +
                       (cac_murata.czz * data_murata.gyro_z);
}

static void ReadDataTdk(void) {
  data_tdk_raw.acc_x_H = ReadSpiTdk(ACCEL_XOUT_H);
  data_tdk_raw.acc_x_L = ReadSpiTdk(ACCEL_XOUT_L);
  data_tdk_raw.acc_y_H = ReadSpiTdk(ACCEL_YOUT_H);
  data_tdk_raw.acc_y_L = ReadSpiTdk(ACCEL_YOUT_L);
  data_tdk_raw.acc_z_H = ReadSpiTdk(ACCEL_ZOUT_H);
  data_tdk_raw.acc_z_L = ReadSpiTdk(ACCEL_ZOUT_L);
  data_tdk_raw.gyro_x_H = ReadSpiTdk(GYRO_XOUT_H);
  data_tdk_raw.gyro_x_L = ReadSpiTdk(GYRO_XOUT_L);
  data_tdk_raw.gyro_y_H = ReadSpiTdk(GYRO_YOUT_H);
  data_tdk_raw.gyro_y_L = ReadSpiTdk(GYRO_YOUT_L);
  data_tdk_raw.gyro_z_H = ReadSpiTdk(GYRO_ZOUT_H);
  data_tdk_raw.gyro_z_L = ReadSpiTdk(GYRO_ZOUT_L);
  data_tdk_raw.temp_H = ReadSpiTdk(TEMP_OUT_H);
  data_tdk_raw.temp_L = ReadSpiTdk(TEMP_OUT_L);
}

static void ConvertDataTdk(void) {
  data_tdk.acc_x = TDK_CONV_ACC(
      CONV_UINT8_INT16(data_tdk_raw.acc_y_H, data_tdk_raw.acc_y_L));
  data_tdk.acc_y = TDK_CONV_ACC(
      CONV_UINT8_INT16(data_tdk_raw.acc_x_H, data_tdk_raw.acc_x_L));
  data_tdk.acc_z = TDK_CONV_ACC(
      CONV_UINT8_INT16(data_tdk_raw.acc_z_H, data_tdk_raw.acc_z_L));
  data_tdk.gyro_x = -TDK_CONV_GYRO(
      CONV_UINT8_INT16(data_tdk_raw.gyro_y_H, data_tdk_raw.gyro_y_L));
  data_tdk.gyro_y = -TDK_CONV_GYRO(
      CONV_UINT8_INT16(data_tdk_raw.gyro_x_H, data_tdk_raw.gyro_x_L));
  data_tdk.gyro_z = -TDK_CONV_GYRO(
      CONV_UINT8_INT16(data_tdk_raw.gyro_z_H, data_tdk_raw.gyro_z_L));
  data_tdk.temp =
      TDK_CONV_TEMP(CONV_UINT8_INT16(data_tdk_raw.temp_H, data_tdk_raw.temp_L));
}

static void InitTdk(void) {
  // The chip must receive power prior to SPI
  EnableSpiTdk();

  // Send 0x81 to PWR_MGMT to initialize SPI
  SendSpiTdk(PWR_MGMT_1, 0x81, false);
  HAL_Delay(100);

  // Setting the sample rates for TDK
  SendSpiTdk(SMPLRT_DIV, 0x00, false);
  SendSpiTdk(CONFIG, 0x00, false);
  SendSpiTdk(GYRO_CONFIG, 0x1A, false);
  SendSpiTdk(ACCEL_CONFIG, 0x18, false);
  SendSpiTdk(ACCEL_CONFIG_2, 0x08, false);
  SendSpiTdk(FIFO_EN, 0xF8, false);
  SendSpiTdk(USER_CTRL, 0x55, false);
}

/* 	CAN_FD Tx data packet:
 *
 * 	BYTES		Contents		Bits of content
 * 	[0..3]		murata_acc_x	[0..31]
 * 	[4..7]		murata_acc_y	[0..31]
 * 	[8..11]		murata_acc_z	[0..31]
 * 	[12..15]	murata_gyro_x	[0..31]
 * 	[16..19]	murata_gyro_y	[0..31]
 * 	[20..23]	murata_gyro_z	[0..31]
 * 	[24..27]	murata_due_temp	[0..31]
 * 	[28..31]	murata_uno_temp	[0..31]
 * 	[32..35]	tdk_acc_x		[0..31]
 * 	[36..39]	tdk_acc_y		[0..31]
 * 	[40..43]	tdk_acc_z		[0..31]
 * 	[44..47]	tdk_gyro_x		[0..31]
 * 	[48..51]	tdk_gyro_y		[0..31]
 * 	[52..55]	tdk_gyro_z		[0..31]
 * 	[56..59]	tdk_temp		[0..31]
 * 	[60..63]	murata_acc_x	[0..31]
 *
 */

static void InitCan(FDCAN_TxHeaderTypeDef *tx_header, uint8_t id) {
  // Initialize the Header
  tx_header->Identifier = id;
  tx_header->IdType = FDCAN_STANDARD_ID;
  tx_header->TxFrameType = FDCAN_DATA_FRAME;
  tx_header->DataLength = FDCAN_DLC_BYTES_8;
  tx_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header->BitRateSwitch = FDCAN_BRS_OFF;
  tx_header->FDFormat = FDCAN_CLASSIC_CAN;
  tx_header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header->MessageMarker = 0x0;  // Ignore because FDCAN_NO_TX_EVENTS

  // Pull standby pin low
  HAL_GPIO_WritePin(FDCAN_STBY_GPIO_Port, FDCAN_STBY_Pin, GPIO_PIN_RESET);

  // Start the FDCAN module
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
