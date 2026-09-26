/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for showTask */
osThreadId_t showTaskHandle;
const osThreadAttr_t showTask_attributes = {
  .name = "showTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint8_t aRxBuffer[20] = {0};
uint8_t rxByte;
uint8_t cmdBuf[20];
volatile uint8_t cmdIndex = 0;
volatile uint8_t cmdReady = 0;
static int32_t ap_moved_cm = 0;    /* net distance the last AP moved, + = forward */   /* net distance the last AC moved, + = forward */
#define AC_MAX_STEP_CM  15         /* AC never drives more than this in one step */

/* ---- IMU / turning ---- */
static float gyro_bias_z = 0.0f;
static uint8_t imu_ok = 0;


static float heading_err = 0.0f;
static uint32_t last_cmd_tick = 0;
#define RUN_GAP_MS  15000/* actual minus commanded heading, deg, + = left */
#define GYRO_SCALE  1.02f          /* calibrate: true angle / gyro angle */
#define HEADING_ERR_MAX  15.0f

/* CALIBRATE THESE (servo angles 0-180, fed to Servo_SetAngle) */
#define SERVO_CENTER   93
#define SERVO_LEFT     0
#define SERVO_RIGHT    180


/* CALIBRATE THESE (PWM, 0..4199) */
#define TURN_SPEED_SLOW    1400
#define TURN_SPEED_MEDIUM  2000
#define TURN_SPEED_FAST    2600

/* CALIBRATE THESE (degrees of overshoot after motors stop) */
#define STOP_OFFSET_LEFT   18.0f
#define STOP_OFFSET_RIGHT  19.0f
#define STOP_OFFSET_SMALL_LEFT   14.0f
#define STOP_OFFSET_SMALL_RIGHT  16.0f
#define STOP_OFFSET_REV_LEFT   18.0f
#define STOP_OFFSET_REV_RIGHT  16.0f
#define SMALL_TURN_THRESHOLD     60.0f

/* set to -1 if a left turn reads negative on gyro Z */
#define GYRO_Z_SIGN_LEFT   (+1)
#define GYRO_DEADBAND   0.3f
#define TURN_LEFT   1
#define TURN_RIGHT  (-1)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of showTask */
  showTaskHandle = osThreadNew(show, NULL, &showTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
The Servo Motor Part
 */
#define ICM_ADDR          (0x68 << 1)   /* if AD0 is pulled high, use (0x69 << 1) */
#define GYRO_SENS_250DPS  131.0f        /* LSB per deg/s at +/-250 dps */

static HAL_StatusTypeDef IMU_RdReg(uint8_t reg, uint8_t *val, uint16_t n)
{
  return HAL_I2C_Mem_Read(&hi2c2, ICM_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, n, 100);
}

static HAL_StatusTypeDef IMU_WrReg(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c2, ICM_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void IMU_SelectBank(uint8_t bank)
{
  IMU_WrReg(0x7F, (uint8_t)(bank << 4));
  HAL_Delay(1);
}

uint8_t IMU_Init(void)
{
  uint8_t who = 0;

  IMU_SelectBank(0);
  if (IMU_RdReg(0x00, &who, 1) != HAL_OK) return 0;   /* no I2C response */
  if (who != 0xEA) return 0;                          /* wrong part */

  IMU_WrReg(0x06, 0x80);            /* PWR_MGMT_1: reset */
  HAL_Delay(100);
  IMU_SelectBank(0);
  IMU_WrReg(0x06, 0x01);            /* wake, auto clock */
  HAL_Delay(20);
  IMU_WrReg(0x07, 0x00);            /* PWR_MGMT_2: accel + gyro on */
  HAL_Delay(20);

  IMU_SelectBank(2);
  IMU_WrReg(0x00, 0x00);            /* GYRO_SMPLRT_DIV = 0 */
  IMU_WrReg(0x01, 0x01);            /* GYRO_CONFIG_1: +/-250 dps, DLPF on */
  HAL_Delay(20);

  IMU_SelectBank(0);
  return 1;
}

float IMU_GetGyroZ_Raw(void)
{
  uint8_t b[2];
  if (IMU_RdReg(0x37, b, 2) != HAL_OK) return 0.0f;   /* GYRO_ZOUT_H/L */
  return (float)(int16_t)((b[0] << 8) | b[1]) / GYRO_SENS_250DPS;
}

void IMU_GetGyroXYZ(float *x, float *y, float *z)
{
  uint8_t b[6];
  if (IMU_RdReg(0x33, b, 6) != HAL_OK) { *x = *y = *z = 0.0f; return; }
  *x = (float)(int16_t)((b[0] << 8) | b[1]) / GYRO_SENS_250DPS;
  *y = (float)(int16_t)((b[2] << 8) | b[3]) / GYRO_SENS_250DPS;
  *z = (float)(int16_t)((b[4] << 8) | b[5]) / GYRO_SENS_250DPS;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (rxByte == '\n' || rxByte == '\r') {
      if (cmdIndex > 0) {
        cmdBuf[cmdIndex] = '\0';
        memcpy(aRxBuffer, cmdBuf, cmdIndex + 1);
        cmdReady = 1;
      }
      cmdIndex = 0;
    } else if (cmdIndex < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdIndex++] = rxByte;
    }
    HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  }
}


void Servo_SetAngle(uint16_t angle)
{
  if (angle > 180) angle = 180;
  uint16_t pulse = 1000 + ((uint32_t)angle * 1000) / 180;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse);
}

#define PWM_MAX 4199

#define CPC_A_x100   7226L    /* -OG 6871L NEW 1601 - 1522 / OG 22.15 NEW 23.31 */
#define CPC_B_x100   7548L    /* - 7548L-OG 7174L 1672 / 22.15 1589 -> 22.15 X -> 23.31*/
// speed: -4199 (full reverse) to +4199 (full forward), 0 = coast
void MotorA_Set(int32_t speed)
{
  if (speed > PWM_MAX) speed = PWM_MAX;
  if (speed < -PWM_MAX) speed = -PWM_MAX;

  if (speed >= 0) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -speed);
  }
}

void MotorB_Set(int32_t speed)
{
  if (speed > PWM_MAX) speed = PWM_MAX;
  if (speed < -PWM_MAX) speed = -PWM_MAX;

  if (speed >= 0) {
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, -speed);
  }
}


#define CTRL_PERIOD_MS   10
#define SPEED_CRUISE     2600
#define SPEED_MIN        2300        /* was 900 - raise until it doesn't stall */
#define DECEL_COUNTS     450
#define KP_SYNC          4
#define MOVE_TIMEOUT_MS  15000
#define ARRIVE_TOL       15          /* counts; ~0.2 cm */
#define STALL_COUNTS     3           /* movement below this = not moving */
#define STALL_MS         300         /* stalled this long -> give up */

static uint32_t encA_prev;
static uint16_t encB_prev;
static int32_t  travA, travB;      /* always positive while moving */

static void Enc_Reset(void)
{
  encA_prev = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
  encB_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  travA = 0;
  travB = 0;
}

/* dir: +1 forward, -1 backward */
static void Enc_Update(int dir)
{
  uint32_t a = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
  uint16_t b = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

  int32_t dA = (int32_t)(a - encA_prev);          /* wraps correctly */
  int16_t dB = (int16_t)(b - encB_prev);          /* wraps correctly */

  encA_prev = a;
  encB_prev = b;

  travA += dir * dA;
  travB += dir * (-(int32_t)dB);
}

/* Drive straight for dist_cm. dir: +1 = FW, -1 = BW. */

/* Drive straight for dist_cm. dir: +1 = FW, -1 = BW.
   Returns 0 = arrived, 1 = timeout, 2 = stalled. */
int Drive_Distance(int32_t dist_cm, int dir)
{
  if (dist_cm <= 0) return 0;

  int32_t tgtA = (dist_cm * CPC_A_x100) / 100;
  int32_t tgtB = (dist_cm * CPC_B_x100) / 100;
  uint32_t elapsed = 0;
  uint32_t stalled = 0;
  int32_t  lastProgress = 0;
  int reason = 0;                      /* 0 arrived, 1 timeout, 2 stall */

  int32_t decel = DECEL_COUNTS;
  if (decel > (tgtA + tgtB) / 4) decel = (tgtA + tgtB) / 4;
  if (decel < 60) decel = 60;

  Servo_SetAngle(SERVO_LEFT);
  osDelay(200);
  Servo_SetAngle(SERVO_CENTER);
  osDelay(250);
  Enc_Reset();

  float yaw = 0.0f;                    /* heading change during this move, + = left */
  uint32_t t_old = HAL_GetTick();

  for (;;)
  {
    Enc_Update(dir);

    /* measure heading drift (no steering) */
    {
      float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
      uint32_t t_now = HAL_GetTick();
      float dt = (t_now - t_old) * 0.001f;
      t_old = t_now;
      gz *= (float)GYRO_Z_SIGN_LEFT;
      if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
      yaw += gz * dt;
    }

    int32_t rem = ((tgtA - travA) + (tgtB - travB)) / 2;
    if (rem <= ARRIVE_TOL) break;
    if (elapsed >= MOVE_TIMEOUT_MS) { reason = 1; break; }

    int32_t progress = (travA + travB) / 2;
    if (progress - lastProgress < STALL_COUNTS) {
      stalled += CTRL_PERIOD_MS;
      if (stalled >= STALL_MS) { reason = 2; break; }
    } else {
      stalled = 0;
    }
    lastProgress = progress;

    int32_t sp = SPEED_CRUISE;
    if (rem < decel) {
      sp = SPEED_MIN + ((SPEED_CRUISE - SPEED_MIN) * rem) / decel;
    }

    int32_t normB = (int32_t)(((int64_t)travB * CPC_A_x100) / CPC_B_x100);
    int32_t corr  = KP_SYNC * (travA - normB);
    int32_t lim   = sp / 3;
    if (corr >  lim) corr =  lim;
    if (corr < -lim) corr = -lim;

    MotorA_Set(dir * (sp - corr));
    MotorB_Set(dir * (sp + corr));

    osDelay(CTRL_PERIOD_MS);
    elapsed += CTRL_PERIOD_MS;
  }

  MotorA_Set(0);
  MotorB_Set(0);

  /* keep measuring while it rolls to a stop */
  for (int i = 0; i < 10; i++)
  {
    float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
    uint32_t t_now = HAL_GetTick();
    float dt = (t_now - t_old) * 0.001f;
    t_old = t_now;
    gz *= (float)GYRO_Z_SIGN_LEFT;
    if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
    yaw += gz * dt;
    osDelay(20);
  }

  heading_err += yaw * GYRO_SCALE;
  if (heading_err >  HEADING_ERR_MAX) heading_err =  HEADING_ERR_MAX;
  if (heading_err < -HEADING_ERR_MAX) heading_err = -HEADING_ERR_MAX;

  return reason;
}

/* ================= Turning ================= */
#define TURN_LOOP_MS    20
#define TURN_TIMEOUT_MS 5000
#define TURN_RATIO      1.71f
#define TURN_PWM_OUTER  4100
#define TURN_PWM_INNER  ((int32_t)(TURN_PWM_OUTER / TURN_RATIO))  /* ≈ 2398 */


void Motor_Brake(void)
{
  /* both channels to full = both inputs high = short the windings */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_MAX);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_MAX);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_MAX);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_MAX);
}

/* ---------- Ultrasonic (HC-SR04) : TRIG=PB15, ECHO=PB14 ---------- */
/* ---------- Ultrasonic (HC-SR04) : TRIG=PB15, ECHO=PB14 ---------- */
void US_Init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  g.Pin = GPIO_PIN_15;               /* TRIG */
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  g.Pin = GPIO_PIN_14;               /* ECHO */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &g);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* one reading in mm, or -1 if no echo */
int32_t US_Read_mm(void)
{
  const uint32_t cpu_us = 168;
  const uint32_t timeout = 30000 * cpu_us;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  uint32_t t0 = DWT->CYCCNT;
  while ((DWT->CYCCNT - t0) < 10 * cpu_us) {}
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  t0 = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
    if ((DWT->CYCCNT - t0) > timeout) return -1;

  uint32_t start = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
    if ((DWT->CYCCNT - start) > timeout) return -1;

  uint32_t us = (DWT->CYCCNT - start) / cpu_us;
  return (int32_t)(us * 10 / 58);
}

/* median of 5 readings in mm, or -1 if too few valid */
int32_t US_Median_mm(void)
{
  int32_t v[5];
  int n = 0;
  for (int i = 0; i < 5; i++) {
    int32_t r = US_Read_mm();
    if (r > 0) v[n++] = r;
    osDelay(30);
  }
  if (n < 3) return -1;
  for (int i = 1; i < n; i++) {
    int32_t k = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[n / 2];
}

void Gyro_CalibrateZBias(void)
{
  float sum = 0.0f;
  for (int i = 0; i < 200; i++) {
    sum += IMU_GetGyroZ_Raw();
    osDelay(5);
  }
  gyro_bias_z = sum / 200.0f;
}

float Car_Turn(float target_deg, int direction, int drive_dir)
{
  float angle = 0.0f;
  float rot = (float)(direction * drive_dir);     /* +1 = nose swings left */

  /* fold in error left over from previous turns */
  float want = target_deg - rot * heading_err;
  if (want < 0.0f) want = 0.0f;

  float offset;
  if (drive_dir < 0)
    offset = (direction == TURN_LEFT) ? STOP_OFFSET_REV_LEFT : STOP_OFFSET_REV_RIGHT;
  else if (target_deg < SMALL_TURN_THRESHOLD)
    offset = (direction == TURN_LEFT) ? STOP_OFFSET_SMALL_LEFT : STOP_OFFSET_SMALL_RIGHT;
  else
    offset = (direction == TURN_LEFT) ? STOP_OFFSET_LEFT : STOP_OFFSET_RIGHT;

  float goal = want - offset;
  uint32_t elapsed = 0;

  if (goal <= 0.0f) return 0.0f;

  Servo_SetAngle((direction == TURN_LEFT) ? SERVO_LEFT : SERVO_RIGHT);
  osDelay(250);

  uint32_t t_old = HAL_GetTick();

  while (angle < goal && elapsed < TURN_TIMEOUT_MS)
  {
    float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
    uint32_t t_now = HAL_GetTick();
    float dt = (t_now - t_old) * 0.001f;
    t_old = t_now;

    gz = gz * (float)GYRO_Z_SIGN_LEFT * (float)direction * (float)drive_dir;
    if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
    angle += gz * dt;

    int32_t inner = drive_dir * TURN_PWM_INNER;
    int32_t outer = drive_dir * TURN_PWM_OUTER;

    if (direction == TURN_LEFT) {
        MotorA_Set(inner);
        MotorB_Set(outer);
    } else {
        MotorA_Set(outer);
        MotorB_Set(inner);
    }

    osDelay(TURN_LOOP_MS);
    elapsed += TURN_LOOP_MS;
  }

  Motor_Brake();
  Servo_SetAngle(SERVO_LEFT);

  /* keep measuring while the car coasts to a stop */
  for (int i = 0; i < 20; i++)
  {
    float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
    uint32_t t_now = HAL_GetTick();
    float dt = (t_now - t_old) * 0.001f;
    t_old = t_now;

    gz = gz * (float)GYRO_Z_SIGN_LEFT * (float)direction * (float)drive_dir;
    if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
    angle += gz * dt;

    osDelay(20);
  }

  Servo_SetAngle(SERVO_CENTER);
  osDelay(200);
  MotorA_Set(0);
  MotorB_Set(0);

  /* update the carried error with what really happened */
  float actual = angle * GYRO_SCALE;
  heading_err += rot * (actual - target_deg);
  if (heading_err >  HEADING_ERR_MAX) heading_err =  HEADING_ERR_MAX;
  if (heading_err < -HEADING_ERR_MAX) heading_err = -HEADING_ERR_MAX;

  return actual;
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    cmdIndex = 0;
    HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  char cmd[20];
  char tx[64];

  osDelay(200);
  imu_ok = IMU_Init();
  {
    int n = sprintf(tx, "IMU %s\r\n", imu_ok ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
  }
  Gyro_CalibrateZBias();
  {
    int n = sprintf(tx, "BIAS %ld (x1000)\r\n", (int32_t)(gyro_bias_z * 1000));
    HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
  }
  Servo_SetAngle(SERVO_CENTER);
  US_Init();
  for (;;)
  {
    if (cmdReady)
    {
      taskENTER_CRITICAL();
      memcpy(cmd, (const void *)aRxBuffer, sizeof(cmd));
      cmdReady = 0;
      {
        uint32_t now = HAL_GetTick();
        if (now - last_cmd_tick > RUN_GAP_MS) heading_err = 0.0f;
        last_cmd_tick = now;
      }
      taskEXIT_CRITICAL();
      cmd[sizeof(cmd) - 1] = '\0';

      /* skip leading junk (stray \n, spaces) */
      char *p = cmd;
      while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

      int dir = 0;
      if ((p[0] == 'F' || p[0] == 'f') && (p[1] == 'W' || p[1] == 'w')) dir = +1;
      if ((p[0] == 'B' || p[0] == 'b') && (p[1] == 'W' || p[1] == 'w')) dir = -1;

      if (dir != 0)
      {
        int32_t d = atoi(&p[2]);
        if (d > 0)
        {
          int why = Drive_Distance(d, dir);
          int n = sprintf(tx, "DONE %s A:%ld B:%ld HE:%ld %s\r\n", p, travA, travB,
                          (int32_t)(heading_err * 10),
                          why == 2 ? "STALL" : (why == 1 ? "TIMEOUT" : "OK"));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
        else
        {
          int n = sprintf(tx, "ERR BADDIST [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'L' || p[0] == 'l') && (p[1] == 'T' || p[1] == 't'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0)
        {
          float got = Car_Turn((float)a, TURN_LEFT, +1);
          int n = sprintf(tx, "DONE %s IMU:%ld\r\n", p, (int32_t)(got * 10));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
        else
        {
          int n = sprintf(tx, "ERR BADANGLE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'R' || p[0] == 'r') && (p[1] == 'T' || p[1] == 't'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0)
        {
          float got = Car_Turn((float)a, TURN_RIGHT, +1);
          int n = sprintf(tx, "DONE %s IMU:%ld\r\n", p, (int32_t)(got * 10));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
        else
        {
          int n = sprintf(tx, "ERR BADANGLE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'S' || p[0] == 's') && (p[1] == 'V' || p[1] == 'v'))
      {
        int a = atoi(&p[2]);
        Servo_SetAngle(a);
        int n = sprintf(tx, "SERVO %d\r\n", a);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'S' || p[0] == 's') && (p[1] == 'P' || p[1] == 'p'))
      {
        int32_t v = atoi(&p[2]);
        MotorA_Set(v);
        MotorB_Set(v);
        osDelay(2000);
        MotorA_Set(0);
        MotorB_Set(0);
        int n = sprintf(tx, "SPD %ld DONE\r\n", v);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'G' || p[0] == 'g') && (p[1] == 'Y' || p[1] == 'y'))
      {
        for (int i = 0; i < 200; i++)
        {
          float gx, gy, gz;
          IMU_GetGyroXYZ(&gx, &gy, &gz);
          int n = sprintf(tx, "X:%ld Y:%ld Z:%ld\r\n",
                          (int32_t)(gx * 10), (int32_t)(gy * 10), (int32_t)(gz * 10));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
          osDelay(50);
        }
      }
      else if ((p[0] == 'B' || p[0] == 'b') && (p[1] == 'I' || p[1] == 'i'))
      {
        Gyro_CalibrateZBias();
        heading_err = 0.0f;
        int n = sprintf(tx, "BIAS %ld (x1000)\r\n", (int32_t)(gyro_bias_z * 1000));
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'Y' || p[0] == 'y') && (p[1] == 'W' || p[1] == 'w'))
      {
        float yaw = 0.0f;
        uint32_t t_old = HAL_GetTick();
        for (int i = 0; i < 1600; i++)
        {
          float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
          uint32_t t_now = HAL_GetTick();
          float dt = (t_now - t_old) * 0.001f;
          t_old = t_now;
          if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
          yaw += gz * dt;
          osDelay(5);
        }
        int n = sprintf(tx, "YAW %ld (x10)\r\n", (int32_t)(yaw * 10));
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'D' || p[0] == 'd') && (p[1] == 'L' || p[1] == 'l'))
      {
        int32_t d = atoi(&p[2]);
        if (d > 0) {
          Car_Turn(45.0f, TURN_LEFT, +1);
          osDelay(200);
          Drive_Distance(d, +1);
          int n = sprintf(tx, "DONE %s\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADDIST [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'D' || p[0] == 'd') && (p[1] == 'R' || p[1] == 'r'))
      {
        int32_t d = atoi(&p[2]);
        if (d > 0) {
          Car_Turn(45.0f, TURN_RIGHT, +1);
          osDelay(200);
          Drive_Distance(d, +1);
          int n = sprintf(tx, "DONE %s\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADDIST [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'X' || p[0] == 'x') && (p[1] == 'L' || p[1] == 'l'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0) {
          float got = Car_Turn((float)a, TURN_LEFT, -1);
          int n = sprintf(tx, "DONE %s IMU:%ld\r\n", p, (int32_t)(got * 10));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADANGLE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'X' || p[0] == 'x') && (p[1] == 'R' || p[1] == 'r'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0) {
          float got = Car_Turn((float)a, TURN_RIGHT, -1);
          int n = sprintf(tx, "DONE %s IMU:%ld\r\n", p, (int32_t)(got * 10));
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADANGLE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'U' || p[0] == 'u') && (p[1] == 'S' || p[1] == 's'))
      {
        int32_t mm = US_Median_mm();
        int n = sprintf(tx, "US %ld mm\r\n", mm);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'A' || p[0] == 'a') && (p[1] == 'C' || p[1] == 'c'))
      {
        int32_t target_mm = atoi(&p[2]) * 10;
        int32_t d = US_Median_mm();
        ap_moved_cm = 0;

        for (int k = 0; k < 4 && d > 0 && d < 1000; k++) {
          int32_t err_mm = d - target_mm;
          if (err_mm > -30 && err_mm < 30) break;
          int32_t cm = (err_mm > 0 ? err_mm : -err_mm) / 10;
          if (cm > AC_MAX_STEP_CM) cm = AC_MAX_STEP_CM;
          int step_dir = (err_mm > 0) ? +1 : -1;
          Drive_Distance(cm, step_dir);
          ap_moved_cm += step_dir * cm;
          d = US_Median_mm();
        }

        int n;
        if (d > 0 && d < 1000)
          n = sprintf(tx, "DONE %s US:%ld MOVED:%ld\r\n", p, d, ap_moved_cm);
        else
          n = sprintf(tx, "DONE %s NOOBJ MOVED:%ld\r\n", p, ap_moved_cm);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'R' || p[0] == 'r') && (p[1] == 'A' || p[1] == 'a'))
      {
        int32_t back = ap_moved_cm;
        if (back > 0)       Drive_Distance(back, -1);
        else if (back < 0)  Drive_Distance(-back, +1);
        ap_moved_cm = 0;
        int n = sprintf(tx, "DONE %s UNDID:%ld\r\n", p, back);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else
      {
        int n = sprintf(tx, "ERR UNKNOWN [%s]\r\n", p);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
    }
    osDelay(20);
  }
  /* USER CODE END 5 */
}
/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the showTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */

  uint8_t hello[32];
  OLED_Init();
  OLED_Clear();
  /* Infinite loop */
  for(;;)
  {

	sprintf((char*)hello, "A:%ld    ", (int32_t)__HAL_TIM_GET_COUNTER(&htim2));
	OLED_ShowString(0, 0, (u8*)hello);

	sprintf((char*)hello, "B:%d    ", (int16_t)__HAL_TIM_GET_COUNTER(&htim3));
	OLED_ShowString(0, 16, (u8*)hello);

	sprintf((char*)hello, "RX:%-10s ", aRxBuffer);
    OLED_ShowString(0, 32, (u8*)hello);

	OLED_Refresh_Gram();
    osDelay(100);

  }


  /* USER CODE END show */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
