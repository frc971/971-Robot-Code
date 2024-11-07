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
#include <time.h>

#include "murata.h"
#include "tdk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc5;

FDCAN_HandleTypeDef hfdcan2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
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
FDCAN_RxHeaderTypeDef rx_header;

static CrossAxisCompMurata
    cac_murata;  // Cross-axis compensation. Details on datasheet page 11
static DataMurata data_murata;
static DataTdk data_tdk;
static uint8_t can_tx[64];
static int can_tx_packet_counter;
static CanData can_out;
static int timer_index = 0;

static DataMurata murata_averaged;
static DataTdk tdk_averaged;

static DataRawInt16 uno_data[IMU_SAMPLES_PER_MS];
static DataRawInt16 due_data[IMU_SAMPLES_PER_MS];
static DataRawInt16 tdk_data[IMU_SAMPLES_PER_MS];

static SpiOut uno_state;
static SpiOut due_state;
static SpiOut tdk_state;

static uint8_t spi1_rx[2];
static uint8_t spi1_tx[2];
static FourBytes spi2_rx;
static FourBytes spi2_tx;
static FourBytes spi3_rx;
static FourBytes spi3_tx;
static FourBytes spi_murata_rx;

// Watchdog
// Watchpuppies, really. When they starve, the main watchdog also starves;
// system reset is triggered on main watchdog timeout.
unsigned long last_can_watchdog_us;
unsigned long last_pwm_watchdog_us;
unsigned long last_spi_uno_watchdog_us;
unsigned long last_spi_due_watchdog_us;
unsigned long last_spi_tdk_watchdog_us;

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
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

static void EnableLeds(void);
static void PowerTdk(void);
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
static void InitCan(FDCAN_TxHeaderTypeDef *tx_header, uint32_t id);
static void PWMSend(TIM_HandleTypeDef *htim, uint32_t channel, float data);
static void ConstructCanfdPacket(uint8_t *tx, DataMurata *murata, DataTdk *tdk);
static void AverageData(void);
static void ComposeData(void);
static void RealignData(void);
static void SpiReadIt(SPI_HandleTypeDef *hspix, uint32_t reg);
static void SpiCsStart(SPI_HandleTypeDef *hspix);
static void SpiCsEnd(SPI_HandleTypeDef *hspix);
static void SpiTdk(DataRawInt16 *data, SPI_HandleTypeDef *hspix, SpiOut *res,
                   SpiIn call);
static void SpiMurata(DataRawInt16 *data, SPI_HandleTypeDef *hspix, SpiOut *res,
                      SpiIn call);

// Watchdog
static uint16_t GetWatchdogTimeoutMs(void);
static void DelayWithWatchdog(uint16_t delay_ms);

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  HAL_IWDG_Refresh(&hiwdg);
  EnableLeds();  // Set LEDs to red
  HAL_IWDG_Refresh(&hiwdg);

  InitCan(&tx_header, 0x01);  // Initialize the CAN module
  HAL_IWDG_Refresh(&hiwdg);

  InitMurata();  // Run the Murata power up sequence (see pg 30 of datasheet)
  HAL_IWDG_Refresh(&hiwdg);

  InitTdk();  // Run the TDK power up sequence (see pg 22 of datasheet)
  HAL_IWDG_Refresh(&hiwdg);

  HAL_TIM_Base_Start_IT(&htim2);  // Start 1 us timer
  HAL_TIM_Base_Start_IT(&htim1);  // Start 1 ms timer
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_IWDG_Refresh(&hiwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Check when CAN, PWM and SPI were last successfully written. Use
    // snapshotted timestamps to prevent race condition from interrupt loop
    // updating during check.
    unsigned long last_can_watchdog_us_copy = last_can_watchdog_us;
    unsigned long last_pwm_watchdog_us_copy = last_pwm_watchdog_us;
    unsigned long last_spi_uno_watchdog_us_copy = last_spi_uno_watchdog_us;
    unsigned long last_spi_due_watchdog_us_copy = last_spi_due_watchdog_us;
    unsigned long last_spi_tdk_watchdog_us_copy = last_spi_tdk_watchdog_us;
    unsigned long current_time_us = __HAL_TIM_GetCounter(&htim2);

    // Feed watchdog. Triggers system reset after 100 ms.
    // We also check 3 sub-watchdogs that monitor the (interrupt) loops for CAN,
    // PWM and SPI. This main watchdog is only fed when all are fed.

    if (current_time_us - last_can_watchdog_us_copy > CAN_WATCHDOG_TIMEOUT_US) {
      // TODO(sindy): add uart logging in future PR -- "CAN watchdog timeout"
    } else if (current_time_us - last_pwm_watchdog_us_copy >
               PWM_WATCHDOG_TIMEOUT_US) {
      // TODO(sindy): add uart logging in future PR -- "PWM watchdog timeout"
    } else if (current_time_us - last_spi_uno_watchdog_us_copy >
               SPI_WATCHDOG_TIMEOUT_US) {
      // TODO(sindy): add uart logging in future PR -- "SPI UNO watchdog
      // timeout"
    } else if (current_time_us - last_spi_due_watchdog_us_copy >
               SPI_WATCHDOG_TIMEOUT_US) {
      // TODO(sindy): add uart logging in future PR -- "SPI DUE watchdog
      // timeout"
    } else if (current_time_us - last_spi_tdk_watchdog_us_copy >
               SPI_WATCHDOG_TIMEOUT_US) {
      // TODO(sindy): add uart logging in future PR -- "SPI TDK watchdog
      // timeout"
    } else {
      HAL_IWDG_Refresh(&hiwdg);
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 |
                                     RCC_OSCILLATORTYPE_LSI |
                                     RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {
  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 200;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 333;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
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
  htim3.Init.Prescaler = 24 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35000;
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

static void PowerTdk(void) {
  // TDK_PWR_EN starts high, must be set low before reading
  HAL_GPIO_WritePin(TDK_PWR_EN_GPIO_Port, TDK_PWR_EN_Pin, GPIO_PIN_RESET);
  // TDK_EN starts high, must be set low after a delay from TDK_PWR_EN

  DelayWithWatchdog(1000);

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
  uint32_t message = (uint32_t)(((address << 2) << 24) | data << 8);
  uint8_t redundancy = GetChecksumMurata(message);
  return (uint32_t)(message | redundancy);
}

static uint32_t MakeSpiReadMsgMurata(uint8_t address) {
  // Constructs SPI read frame
  return MakeSpiMsgMurata(address, 0x0000);
}

static uint32_t TransmitSpiMurata(uint32_t value, GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin, SPI_HandleTypeDef *hspix) {
  FourBytes rx_data_raw;
  FourBytes rx_data;
  FourBytes tx_data;
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

static bool InitMurata(void) {
  int num_attempts = 0;
  uint32_t response_due = 0;
  uint32_t response_uno = 0;
  bool due_ok = false;
  bool uno_ok = false;

  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin,
                    GPIO_PIN_RESET);  // Reset UNO
  DelayWithWatchdog(1);
  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  DelayWithWatchdog(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);

  DelayWithWatchdog(25);

  SendSpiDue(MakeSpiReadMsgMurata(ACC_DC1_ADDRESS));  // cxx_cxy address

  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiUno(WRITE_OP_MODE_NORMAL);
  DelayWithWatchdog(70);

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
  DelayWithWatchdog(1);
  HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  DelayWithWatchdog(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);

  DelayWithWatchdog(25);

  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiUno(WRITE_OP_MODE_NORMAL);
  DelayWithWatchdog(70);

  SendSpiUno(
      MakeSpiMsgMurata(MURATA_GYRO_FILTER_ADDR, MURATA_GYRO_FILTER_300HZ));
  SendSpiUno(MakeSpiMsgMurata(MURATA_ACC_FILTER_ADDR, MURATA_ACC_FILTER_300HZ));

  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                    GPIO_PIN_RESET);  // Reset DUE
  DelayWithWatchdog(1);
  HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);
  DelayWithWatchdog(25);

  SendSpiDue(WRITE_OP_MODE_NORMAL);
  SendSpiDue(WRITE_OP_MODE_NORMAL);
  DelayWithWatchdog(1);

  SendSpiDue(
      MakeSpiMsgMurata(MURATA_GYRO_FILTER_ADDR, MURATA_GYRO_FILTER_300HZ));

  for (num_attempts = 0; num_attempts < 2; num_attempts++) {
    DelayWithWatchdog(405);

    SendSpiDue(WRITE_EOI_BIT);
    SendSpiUno(WRITE_EOI_BIT);

    /* UNO */
    SendSpiUno(READ_SUMMARY_STATUS);
    SendSpiUno(READ_SUMMARY_STATUS);
    DelayWithWatchdog(3);
    response_uno = SendSpiUno(READ_SUMMARY_STATUS);
    uno_ok = !CHECK_RS_ERROR(response_uno);

    /* DUE */
    SendSpiDue(READ_SUMMARY_STATUS);
    SendSpiDue(READ_SUMMARY_STATUS);
    DelayWithWatchdog(3);
    response_due = SendSpiDue(READ_SUMMARY_STATUS);
    due_ok = !CHECK_RS_ERROR(response_due);

    if ((due_ok == false || uno_ok == false) && (num_attempts == 0)) {
      HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin,
                        GPIO_PIN_RESET);  // Reset UNO
      DelayWithWatchdog(1);
      HAL_GPIO_WritePin(RESET_UNO_GPIO_Port, RESET_UNO_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin,
                        GPIO_PIN_RESET);  // Reset DUE
      DelayWithWatchdog(1);
      HAL_GPIO_WritePin(RESET_DUE_GPIO_Port, RESET_DUE_Pin, GPIO_PIN_SET);
      DelayWithWatchdog(25);

      SendSpiDue(WRITE_OP_MODE_NORMAL);
      SendSpiDue(WRITE_OP_MODE_NORMAL);
      SendSpiUno(WRITE_OP_MODE_NORMAL);
      DelayWithWatchdog(50);

      SendSpiUno(
          MakeSpiMsgMurata(MURATA_GYRO_FILTER_ADDR, MURATA_GYRO_FILTER_300HZ));
      SendSpiUno(
          MakeSpiMsgMurata(MURATA_ACC_FILTER_ADDR, MURATA_ACC_FILTER_300HZ));
      SendSpiDue(
          MakeSpiMsgMurata(MURATA_GYRO_FILTER_ADDR, MURATA_GYRO_FILTER_300HZ));
      DelayWithWatchdog(45);
    } else {
      break;
    }
  }

  if (!due_ok || !uno_ok) {
    return false;
  }

  return true;
}

static void InitTdk(void) {
  // Power up sequence. See datasheet p22 for details
  // The chip must receive power prior to SPI
  PowerTdk();

  // Send 0x81 to PWR_MGMT to initialize SPI
  SendSpiTdk(PWR_MGMT_1, 0x81, false);
  DelayWithWatchdog(100);

  // Setting the sample rates for TDK
  SendSpiTdk(USER_CTRL, 0x55, false);
  SendSpiTdk(ACCEL_CONFIG, 0x18, false);
  SendSpiTdk(ACCEL_CONFIG_2, 0x08, false);
  SendSpiTdk(GYRO_CONFIG, 0x1A, false);
  SendSpiTdk(FIFO_EN, 0x00, false);
  SendSpiTdk(CONFIG, 0x00, false);
  SendSpiTdk(SMPLRT_DIV, 0x00, false);
}

static void InitCan(FDCAN_TxHeaderTypeDef *tx_header, uint32_t id) {
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

  // Initialize can_out to zeros
  memset(&can_out, 0, sizeof(can_out));
}

static void PWMSend(TIM_HandleTypeDef *htim, uint32_t channel, float data) {
  if (htim == &htim3) {
    // 10% pwm == min, 90% pwm == max
    float pwm_period = (float)(htim->Init.Period);
    float range;
    float scale;
    float translated;
    uint32_t result;

    switch (channel) {
      case TIM_CHANNEL_3:
        // Sends TDK Z-gyro data over PWM_Rate port
        range = 2 * TDK_GYRO_MAG_MAX;
        scale = (pwm_period * .8f) / range;
        translated = data + range * .5f;
        result = (uint32_t)(scale * translated + pwm_period * .1f);
        if (result < pwm_period * .1f) {
          result = pwm_period * .1f;
        }
        if (result > pwm_period * .9f) {
          result = pwm_period * .9f;
        }
        TIM3->CCR3 = result;
        last_pwm_watchdog_us = __HAL_TIM_GetCounter(&htim2);
        break;

      case TIM_CHANNEL_4:
        // Unused PWM_Heading port
        // TODO: (Zach) figure out if we keep it with the
        // same as CH3 or if we add something else
        range = 2 * TDK_GYRO_MAG_MAX;
        scale = (pwm_period * .8f) / range;
        translated = data + range * .5f;
        result = (uint32_t)(scale * translated + pwm_period * .1f);
        if (result < pwm_period * .1f) {
          result = pwm_period * .1f;
        }
        if (result > pwm_period * .9f) {
          result = pwm_period * .9f;
        }
        TIM3->CCR4 = result;
        last_pwm_watchdog_us = __HAL_TIM_GetCounter(&htim2);
        break;

      default:
        break;
    }
  }
}

/* 	CAN_FD Tx data packet specs:
 *  https://docs.google.com/document/d/12AJUruW7DZ2pIrDzTyPC0qqFoia4QOSVlax6Jd7m4H0
 */

static void ConstructCanfdPacket(uint8_t *tx, DataMurata *murata,
                                 DataTdk *tdk) {
  // Clear the CAN packet
  memset(tx, 0, 64 * sizeof(*tx));

  // Write in the struct data
  can_out.tdk_acc_x = tdk->acc_x;
  can_out.tdk_acc_y = tdk->acc_y;
  can_out.tdk_acc_z = tdk->acc_z;
  can_out.tdk_gyro_x = tdk->gyro_x;
  can_out.tdk_gyro_y = tdk->gyro_y;
  can_out.tdk_gyro_z = tdk->gyro_z;

  can_out.murata_acc_x = murata->acc_x;
  can_out.murata_acc_y = murata->acc_y;
  can_out.murata_acc_z = murata->acc_z;
  can_out.murata_gyro_x = murata->gyro_x;
  can_out.murata_gyro_y = murata->gyro_y;
  can_out.murata_gyro_z = murata->gyro_z;

  if (tdk->temp < 0) {
    can_out.tdk_temp = 0;
  } else if (tdk->temp > 255) {
    can_out.tdk_temp = 255;
  } else {
    can_out.tdk_temp = (uint8_t)(tdk->temp);
  }

  if (murata->uno_temp < 0) {
    can_out.uno_temp = 0;
  } else if (murata->uno_temp > 255) {
    can_out.uno_temp = 255;
  } else {
    can_out.uno_temp = (uint8_t)(murata->uno_temp);
  }

  if (murata->due_temp < 0) {
    can_out.due_temp = 0;
  } else if (murata->due_temp > 255) {
    can_out.due_temp = 255;
  } else {
    can_out.due_temp = (uint8_t)(murata->due_temp);
  }

  can_out.timestamp = __HAL_TIM_GetCounter(&htim2);

  memcpy(&tx[0], &can_out, sizeof(can_out));
}

static void AverageData(void) {
  // Clear the float data fields
  memset(&tdk_averaged, 0, sizeof(tdk_averaged));
  memset(&murata_averaged, 0, sizeof(murata_averaged));

  for (int i = 0; i < IMU_SAMPLES_PER_MS; i++) {
    tdk_averaged.acc_x += tdk_data[i].acc_x;
    tdk_averaged.acc_y += tdk_data[i].acc_y;
    tdk_averaged.acc_z += tdk_data[i].acc_z;
    tdk_averaged.gyro_x += tdk_data[i].gyro_x;
    tdk_averaged.gyro_y += tdk_data[i].gyro_y;
    tdk_averaged.gyro_z += tdk_data[i].gyro_z;
    tdk_averaged.temp += tdk_data[i].temp;

    murata_averaged.acc_x += uno_data[i].acc_x;
    murata_averaged.acc_y += uno_data[i].acc_y;
    murata_averaged.acc_z += uno_data[i].acc_z;
    murata_averaged.gyro_x += uno_data[i].gyro_x;
    murata_averaged.gyro_y += due_data[i].gyro_y;
    murata_averaged.gyro_z += due_data[i].gyro_z;
    murata_averaged.uno_temp += uno_data[i].temp;
    murata_averaged.due_temp += due_data[i].temp;
  }

  tdk_averaged.acc_x /= IMU_SAMPLES_PER_MS;
  tdk_averaged.acc_y /= IMU_SAMPLES_PER_MS;
  tdk_averaged.acc_z /= IMU_SAMPLES_PER_MS;
  tdk_averaged.gyro_x /= IMU_SAMPLES_PER_MS;
  tdk_averaged.gyro_y /= IMU_SAMPLES_PER_MS;
  tdk_averaged.gyro_z /= IMU_SAMPLES_PER_MS;
  tdk_averaged.temp /= IMU_SAMPLES_PER_MS;

  murata_averaged.acc_x /= IMU_SAMPLES_PER_MS;
  murata_averaged.acc_y /= IMU_SAMPLES_PER_MS;
  murata_averaged.acc_z /= IMU_SAMPLES_PER_MS;
  murata_averaged.gyro_x /= IMU_SAMPLES_PER_MS;
  murata_averaged.gyro_y /= IMU_SAMPLES_PER_MS;
  murata_averaged.gyro_z /= IMU_SAMPLES_PER_MS;
  murata_averaged.uno_temp /= IMU_SAMPLES_PER_MS;
  murata_averaged.due_temp /= IMU_SAMPLES_PER_MS;
}

static void ComposeData(void) {
  // Clear the float data fields
  memset(&data_tdk, 0, sizeof(data_tdk));
  memset(&data_murata, 0, sizeof(data_murata));

  // Assign the converted binary
  data_tdk.acc_x = TDK_CONV_ACC(tdk_averaged.acc_x);
  data_tdk.acc_y = TDK_CONV_ACC(tdk_averaged.acc_y);
  data_tdk.acc_z = TDK_CONV_ACC(tdk_averaged.acc_z);
  data_tdk.gyro_x = TDK_CONV_GYRO(tdk_averaged.gyro_x);
  data_tdk.gyro_y = TDK_CONV_GYRO(tdk_averaged.gyro_y);
  data_tdk.gyro_z = TDK_CONV_GYRO(tdk_averaged.gyro_z);
  data_tdk.temp = TDK_CONV_TEMP(tdk_averaged.temp);

  data_murata.acc_x = MURATA_CONV_ACC(murata_averaged.acc_x);
  data_murata.acc_y = MURATA_CONV_ACC(murata_averaged.acc_y);
  data_murata.acc_z = MURATA_CONV_ACC(murata_averaged.acc_z);
  data_murata.gyro_x = MURATA_CONV_GYRO(murata_averaged.gyro_x);
  data_murata.gyro_y = MURATA_CONV_GYRO(murata_averaged.gyro_y);
  data_murata.gyro_z = MURATA_CONV_GYRO(murata_averaged.gyro_z);
  data_murata.uno_temp = MURATA_CONV_TEMP(murata_averaged.uno_temp);
  data_murata.due_temp = MURATA_CONV_TEMP(murata_averaged.due_temp);

  // Apply the murata cross-axis compensation
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

static void RealignData(void) {
  // Rotate the axis based on observed behavior
  float f = data_tdk.acc_x;
  data_tdk.acc_x = -data_tdk.acc_y;
  data_tdk.acc_y = -f;
  f = data_tdk.gyro_x;
  data_tdk.acc_z = -data_tdk.acc_z;
  data_tdk.gyro_x = -data_tdk.gyro_y;
  data_tdk.gyro_y = -f;
  data_tdk.gyro_z = -data_tdk.gyro_z;

  f = data_murata.acc_x;
  data_murata.acc_x = -data_murata.acc_y;
  data_murata.acc_y = -f;
  f = data_murata.gyro_x;
  data_murata.gyro_x = -data_murata.gyro_y;
  data_murata.gyro_y = -f;
}

// Read SPI in interrupt mode
static void SpiReadIt(SPI_HandleTypeDef *hspix, uint32_t reg) {
  if (hspix == &hspi1) {
    spi1_tx[0] = 0x00;
    spi1_tx[1] = (uint8_t)(reg & 0xFF) | 0x80;
    memset(spi1_rx, 0, sizeof(*spi1_rx));

    HAL_SPI_TransmitReceive_IT(hspix, spi1_tx, spi1_rx, sizeof(spi1_tx) / 2);
  } else if (hspix == &hspi2 || hspix == &hspi3) {
    ConvertEndianMurata((hspix == &hspi2) ? (spi2_tx.byte) : (spi3_tx.byte),
                        reg);
    memset((hspix == &hspi2) ? spi2_rx.byte : spi3_rx.byte, 0, 4);

    HAL_SPI_TransmitReceive_IT(
        hspix, (hspix == &hspi2) ? spi2_tx.byte : spi3_tx.byte,
        (hspix == &hspi2) ? spi2_rx.byte : spi3_rx.byte, 2);
  }
}

static void SpiCsStart(SPI_HandleTypeDef *hspix) {
  if (hspix == &hspi1) {
    HAL_GPIO_WritePin(CS_TDK_ST_GPIO_Port, CS_TDK_ST_Pin,
                      GPIO_PIN_RESET);  // CS low at start of transmission
  } else if (hspix == &hspi2) {
    HAL_GPIO_WritePin(CS_UNO_GPIO_Port, CS_UNO_Pin,
                      GPIO_PIN_RESET);  // CS low at start of transmission
  } else if (hspix == &hspi3) {
    HAL_GPIO_WritePin(CS_DUE_GPIO_Port, CS_DUE_Pin,
                      GPIO_PIN_RESET);  // CS low at start of transmission
  }
}

static void SpiCsEnd(SPI_HandleTypeDef *hspix) {
  if (hspix == &hspi1) {
    HAL_GPIO_WritePin(CS_TDK_ST_GPIO_Port, CS_TDK_ST_Pin,
                      GPIO_PIN_SET);  // CS high at end of transmission
  } else if (hspix == &hspi2) {
    HAL_GPIO_WritePin(CS_UNO_GPIO_Port, CS_UNO_Pin,
                      GPIO_PIN_SET);  // CS high at end of transmission
  } else if (hspix == &hspi3) {
    HAL_GPIO_WritePin(CS_DUE_GPIO_Port, CS_DUE_Pin,
                      GPIO_PIN_SET);  // CS high at end of transmission
  }
}

static void SpiTdk(DataRawInt16 *data, SPI_HandleTypeDef *hspix, SpiOut *res,
                   SpiIn call) {
  switch (call) {
    case SPI_INIT:
      InitTdk();
      break;
    case SPI_ZERO:
      memset(data, 0, IMU_SAMPLES_PER_MS * sizeof(*data));
      break;
    case SPI_START:
      data[timer_index].state = 0;
      data[timer_index].index = 0;
      *res = SPI_READY;
      break;

    case SPI_RUN:
      *res = SPI_BUSY;

      SpiCsStart(hspix);

      switch (data[timer_index].state) {
        case 0:
          SpiReadIt(hspix, ACCEL_XOUT_H);  // REG + acc_x_h
          break;
        case 1:
          SpiReadIt(hspix, 0x00);  // acc_x_l + acc_y_h
          break;
        case 2:
          SpiReadIt(hspix, 0x00);  // acc_y_l + acc_z_h
          break;
        case 3:
          SpiReadIt(hspix, 0x00);  // acc_z_l + temp_h
          break;
        case 4:
          SpiReadIt(hspix, 0x00);  // temp_l + gyro_x_h
          break;
        case 5:
          SpiReadIt(hspix, 0x00);  // gyro_x_l + gyro_y_h
          break;
        case 6:
          SpiReadIt(hspix, 0x00);  // gyro_y_l + gyro_z_h
          break;
        case 7:
          SpiReadIt(hspix, 0x00);  // gyro_z_l + UNUSED
          break;
      }

      *res = SPI_READY;
      last_spi_tdk_watchdog_us = __HAL_TIM_GetCounter(&htim2);
      break;
  }
}

static void SpiMurata(DataRawInt16 *data, SPI_HandleTypeDef *hspix, SpiOut *res,
                      SpiIn call) {
  switch (call) {
    case SPI_INIT:
      InitMurata();
      break;
    case SPI_ZERO:
      memset(data, 0, IMU_SAMPLES_PER_MS * sizeof(*data));
      break;
    case SPI_START:
      data[timer_index].state = (hspix == &hspi3) ? 4 : 0;
      data[timer_index].index = 0;
      *res = SPI_READY;
      break;

    case SPI_RUN:
      *res = SPI_BUSY;

      SpiCsStart(hspix);

      switch (data[timer_index].state) {
        case 0:
          SpiReadIt(hspix, READ_ACC_X);
          break;
        case 1:
          SpiReadIt(hspix, READ_ACC_Y);
          break;
        case 2:
          SpiReadIt(hspix, READ_ACC_Z);
          break;
        case 3:
          SpiReadIt(hspix, READ_GYRO_X);
          break;
        case 4:
          SpiReadIt(hspix, READ_TEMP);
          break;
        case 5:
          SpiReadIt(hspix, READ_GYRO_Y);
          break;
        case 6:
          SpiReadIt(hspix, READ_GYRO_Z);
          break;
      }

      *res = SPI_READY;
      last_spi_uno_watchdog_us = __HAL_TIM_GetCounter(&htim2);
      last_spi_due_watchdog_us = __HAL_TIM_GetCounter(&htim2);
      break;
  }
}

static uint16_t GetWatchdogTimeoutMs(void) {
  return (uint16_t)((uint32_t)(1000) * hiwdg.Init.Reload *
                    hiwdg.Init.Prescaler / LSI_CLOCK_FREQ_HZ);
}

static void DelayWithWatchdog(uint16_t delay_ms) {
  // Delay without triggering watchdog reset. We do this by feeding the watchdog
  // at least once during this interval; here we arbitrarily choose do it 3
  // times. Note that this function will cause a delay of slightly longer than
  // the requested due to nonzero compute time.
  uint16_t watchdog_pet_period_ms = GetWatchdogTimeoutMs() / 3;
  uint16_t remaining_delay_time_ms = delay_ms;
  while (watchdog_pet_period_ms < remaining_delay_time_ms) {
    HAL_Delay(watchdog_pet_period_ms);
    HAL_IWDG_Refresh(&hiwdg);
    remaining_delay_time_ms -= watchdog_pet_period_ms;
  }
  HAL_Delay(remaining_delay_time_ms);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    timer_index++;
    timer_index %= IMU_SAMPLES_PER_MS;

    if (timer_index == 0) {
      can_tx_packet_counter = 0;
      AverageData();
      ComposeData();
      RealignData();

      ConstructCanfdPacket(can_tx, &data_murata, &data_tdk);
      can_out.can_counter++;
      can_out.can_counter =
          (can_out.can_counter == 0xFFFF) ? 0 : can_out.can_counter;

      PWMSend(&htim3, TIM_CHANNEL_3, (data_murata.gyro_z));
      PWMSend(&htim3, TIM_CHANNEL_4, (data_murata.gyro_z));

      SpiTdk(tdk_data, &hspi1, &tdk_state, SPI_ZERO);
      SpiMurata(uno_data, &hspi2, &uno_state, SPI_ZERO);
      SpiMurata(due_data, &hspi3, &due_state, SPI_ZERO);
    }

    SpiTdk(tdk_data, &hspi1, &tdk_state, SPI_START);
    SpiMurata(uno_data, &hspi2, &uno_state, SPI_START);
    SpiMurata(due_data, &hspi3, &due_state, SPI_START);

    if (tdk_state == SPI_READY && uno_state == SPI_READY &&
        due_state == SPI_READY) {
      for (int j = 0; j < 3; j++) {
        tx_header.Identifier = can_tx_packet_counter + j + 1;
        if (tx_header.Identifier == 9) {
          break;
        }
        while (HAL_FDCAN_AddMessageToTxFifoQ(
                   &hfdcan2, &tx_header,
                   can_tx + (can_tx_packet_counter + j) * 8) != HAL_OK) {
          // Error Handler will stop execution
        }
      }
      can_tx_packet_counter += 3;
      last_can_watchdog_us = __HAL_TIM_GetCounter(&htim2);

      SpiTdk(tdk_data, &hspi1, &tdk_state, SPI_RUN);
      SpiMurata(uno_data, &hspi2, &uno_state, SPI_RUN);
      SpiMurata(due_data, &hspi3, &due_state, SPI_RUN);
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    switch (tdk_data[timer_index].state) {
      case 0:
        tdk_data[timer_index].acc_x = (int16_t)(spi1_rx[0] << 8);
        break;
      case 1:
        tdk_data[timer_index].acc_x |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].acc_y = (int16_t)(spi1_rx[0] << 8);
        break;
      case 2:
        tdk_data[timer_index].acc_y |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].acc_z = (int16_t)(spi1_rx[0] << 8);
        break;
      case 3:
        tdk_data[timer_index].acc_z |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].temp = (int16_t)(spi1_rx[0] << 8);
        break;
      case 4:
        tdk_data[timer_index].temp |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].gyro_x = (int16_t)(spi1_rx[0] << 8);
        break;
      case 5:
        tdk_data[timer_index].gyro_x |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].gyro_y = (int16_t)(spi1_rx[0] << 8);
        break;
      case 6:
        tdk_data[timer_index].gyro_y |= (int16_t)spi1_rx[1];
        tdk_data[timer_index].gyro_z = (int16_t)(spi1_rx[0] << 8);
        break;
      case 7:
        tdk_data[timer_index].gyro_z |= (int16_t)spi1_rx[1];
        break;
      default:
        SpiCsEnd(hspi);
        return;
    }

    tdk_data[timer_index].state++;
    if (tdk_data[timer_index].state >= 8) {
      SpiCsEnd(hspi);
      tdk_data[timer_index].state = 0;
      can_out.tdk_counter++;
      can_out.tdk_counter =
          (can_out.tdk_counter == 0xFFFF) ? 0 : can_out.tdk_counter;
      return;
    }

    SpiTdk(tdk_data, hspi, &tdk_state, SPI_RUN);
    return;

  } else if (hspi == &hspi2 || hspi == &hspi3) {
    SpiCsEnd(hspi);
    ConvertEndianMurata(spi_murata_rx.byte, (hspi == &hspi2)
                                                ? spi2_rx.four_bytes
                                                : spi3_rx.four_bytes);

    if (hspi == &hspi2) {
      switch (uno_data[timer_index].state) {
        case 0:
          uno_data[timer_index].temp =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 1:
          uno_data[timer_index].acc_x =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 2:
          uno_data[timer_index].acc_y =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 3:
          uno_data[timer_index].acc_z =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 4:
          uno_data[timer_index].gyro_x =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        default:
          return;
      }

      if (uno_data[timer_index].index == 1) {
        uno_data[timer_index].index = 0;

        can_out.uno_counter++;
        can_out.uno_counter =
            (can_out.uno_counter == 0xFFFF) ? 0 : can_out.uno_counter;

        return;
      }

      uno_data[timer_index].state++;

      if (uno_data[timer_index].state == 5) {
        uno_data[timer_index].state = 0;
        uno_data[timer_index].index++;
      }

      SpiMurata(uno_data, hspi, &uno_state, SPI_RUN);

      return;
    } else if (hspi == &hspi3) {
      switch (due_data[timer_index].state - 4) {
        case 0:
          due_data[timer_index].gyro_z =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 1:
          due_data[timer_index].temp =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        case 2:
          due_data[timer_index].gyro_y =
              GET_SPI_DATA_INT16(spi_murata_rx.four_bytes);
          break;
        default:
          return;
      }

      if (due_data[timer_index].index == 1) {
        due_data[timer_index].index = 0;

        can_out.due_counter++;
        can_out.due_counter =
            (can_out.due_counter == 0xFFFF) ? 0 : can_out.due_counter;

        return;
      }

      due_data[timer_index].state++;

      if (due_data[timer_index].state == 7) {
        due_data[timer_index].state = 4;
        due_data[timer_index].index++;
      }

      SpiMurata(due_data, hspi, &due_state, SPI_RUN);

      return;
    }
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
