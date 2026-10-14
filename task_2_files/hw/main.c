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
#include <ctype.h>
#include <errno.h>
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
/* TASK 2 MERGE: group hardware/calibration, friend-inspired turn mechanism.
 * See Task2_README.md for protocol, calibration and validation limits.
 * This is a replacement main.c inside the group's existing STM32 project.
 * It does not contain a hard-coded autonomous route or image recognition.
 * RPi UART3: 115200 baud, 8N1. Send ASCII commands ending in LF (or CRLF).
 * Examples: FW30, BW10, LT10, LT45, LT90, RT10, RT45, RT90, XL45, XR90,
 *           US, IR, AC30, BI, STOP. Distances in commands are centimetres;
 *           US/IR readings are millimetres. IMU/W/TRIM/HE are degrees x10.
 * Wait for each reply before the next command. US/IR sent during motion are
 * queued and answered AFTER that motion. STOP/ST preempts motion cooperatively.
 * DONE means the command has finished; inspect STATUS (or final FW/BW word).
 * AC30 drives until the front ultrasonic distance is 30 cm +/-20 mm.
 * IERN150 drives FW up to 150 cm and stops at the edge seen by the RIGHT IR (see IE_Check).
 * No travel-distance cap; 60 s timeout plus sensor/gyro/stall/STOP protection.
 * New tuning: TM1 single-wheel turn, TM0 original wheel ratio, VS2600 cruise.
 * Permanent values: center_us, STEER_*_US_DEFAULT, CPC_*_x100, GYRO_SCALE,
 * IR_K/IR_OFFSET, SPEED_CRUISE, TRIM_PWM and TASK2_* definitions below.
 * New mechanism requires floor tests; preserved stop constants are only
 * starting values after changing wheel behaviour. No claim of hardware tuning.
 */
uint8_t aRxBuffer[20] = {0};
uint8_t rxByte;
uint8_t cmdBuf[20];
volatile uint8_t cmdIndex = 0;
volatile uint8_t cmdReady = 0; /* retained for source compatibility */
#define RX_QUEUE_SLOTS 8
static char rx_queue[RX_QUEUE_SLOTS][20];
static volatile uint8_t rx_head = 0, rx_tail = 0;
static volatile uint8_t rx_rejected = 0;
static uint8_t rx_discard = 0;
static volatile uint8_t stop_requested = 0;
static uint8_t gyro_ready = 0;
enum { MOTION_OK, MOTION_TIMEOUT, MOTION_STALL, MOTION_STOPPED,
       MOTION_IMU, MOTION_ANGLE, MOTION_BADARG, MOTION_NOOBJ };
static int motion_status = MOTION_OK;
static const char *Motion_Status(void)
{
  switch (motion_status) {
    case MOTION_OK: return "OK";
    case MOTION_TIMEOUT: return "TIMEOUT";
    case MOTION_STALL: return "STALL";
    case MOTION_STOPPED: return "STOPPED";
    case MOTION_IMU: return "IMUFAIL";
    case MOTION_ANGLE: return "ANGLE_ERROR";
    case MOTION_NOOBJ: return "NOOBJ";
    default: return "BADARG";
  }
}
static int32_t ap_moved_cm = 0; /* net distance of last AC, rounded cm */
#define AC_TOL_MM        20        /* AC stops when within +/- 2 cm of target */

/* ---- IMU / turning ---- */
static float gyro_bias_z = 0.0f;
static uint8_t imu_ok = 0;


static float heading_err = 0.0f;
static uint32_t last_cmd_tick = 0;
#define RUN_GAP_MS  15000/* actual minus commanded heading, deg, + = left */
#define GYRO_SCALE  1.006f         /* calibrate: true angle / gyro angle.
                                      2 Oct 2026: hand-rotation YW test, 5 runs of 360 deg:
                                      YAW -3574..-3582 -> 3600/3580 = 1.006.
                                      (Earlier 0.95 was wrong: caused by unmeasured servo-swing
                                      rotation, now counted in Car_Turn.) */
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
/* TODO: RECALIBRATE all STOP_OFFSET_* below. They were tuned for the old
   1000/2000 us steering lock; lock is now 950/2030 (STEER_*_US_DEFAULT).
   Method: BI, then the turn ~5x; new = old + (avg IMU angle - target).
   Do the 6x RT90 drift test too (check GYRO_SCALE).
   DONE: GYRO_SCALE (YW hand test), STOP_OFFSET_RIGHT, STOP_OFFSET_LEFT.
   REV_LEFT/RIGHT done (4x each, BI each).
   STILL TO DO: SMALL_LEFT/RIGHT (only if the planner uses them). */
#define STOP_OFFSET_LEFT   14.7f   /* 2 Oct 2026: 18 -> 14.7. 4x LT90 @ KL950 (BI each):
                                      87.0 87.6 85.5 86.6, avg 86.7 -> 18 - 3.3 */
#define STOP_OFFSET_RIGHT  14.0f   /* full speed. Only used when ADAPTIVE_STOP is 0.
                                      Needed 14 at ~125 deg/s but ~18.6 at ~155 deg/s (battery). */
#define STOP_OFFSET_SMALL_LEFT   14.0f
#define STOP_OFFSET_SMALL_RIGHT  16.0f
#define STOP_OFFSET_REV_LEFT   4.2f    /* 2 Oct 2026: 18 -> 14.7 (full speed) -> 4.2 at TURN_SPEED_PCT_XL 75:
                                      4x XL90 fell short of their aim by 10.8 10.2 10.9 10.0, avg 10.5 */
#define STOP_OFFSET_REV_RIGHT  4.8f    /* 2 Oct 2026: 16 -> 15.3 (full speed) -> 4.8 at TURN_SPEED_PCT_XR 75:
                                      5x XR90 fell short of their aim by 11.0 10.5 10.8 10.5 9.6, avg 10.5 */
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

/* Gyro health counters (reset at the start of each Car_Turn,
   printed in the DONE reply of LT/RT/XL/XR). */
volatile uint16_t imu_err_count = 0;   /* failed I2C reads */
volatile uint16_t imu_sat_count = 0;   /* readings at the +/-250 dps limit */
volatile float    imu_max_dps   = 0.0f;/* largest |rate| seen */
volatile float    turn_brake_rate = 0.0f;/* spin rate (deg/s) when the last turn braked */
volatile float    turn_want  = 0.0f;     /* angle the last turn aimed for, after carry-over (deg) */
volatile float    turn_trim  = 0.0f;     /* angle the post-turn trim added, + = more turn (deg) */

float IMU_GetGyroZ_Raw(void)
{
  static float last_good = 0.0f;
  uint8_t b[2];
  if (IMU_RdReg(0x37, b, 2) != HAL_OK) {               /* GYRO_ZOUT_H/L */
    imu_err_count++;
    return last_good;          /* was 0.0f: a failed read lost that slice of rotation */
  }
  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  if (raw >= 32767 || raw <= -32768) imu_sat_count++;
  last_good = (float)raw / GYRO_SENS_250DPS;
  float a = (last_good < 0.0f) ? -last_good : last_good;
  if (a > imu_max_dps) imu_max_dps = a;
  return last_good;
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
  if (huart->Instance != USART3) return;
  if (rxByte == '\n' || rxByte == '\r') {
    if (!rx_discard && cmdIndex > 0) {
      cmdBuf[cmdIndex] = '\0';
      for (uint8_t i = 0; i < cmdIndex; i++) cmdBuf[i] = (uint8_t)toupper(cmdBuf[i]);
      if (strcmp((char *)cmdBuf, "STOP") == 0 || strcmp((char *)cmdBuf, "ST") == 0) {
        stop_requested = 1;
        rx_tail = rx_head; /* discard pending moves on STOP */
      } else if (!stop_requested) {
        uint8_t next = (uint8_t)((rx_head + 1U) % RX_QUEUE_SLOTS);
        if (next == rx_tail) rx_rejected = 1;
        else {
          memcpy(rx_queue[rx_head], cmdBuf, cmdIndex + 1U);
          rx_head = next;
        }
      } else rx_rejected = 1;
    }
    if (rx_discard) rx_rejected = 1;
    cmdIndex = 0; rx_discard = 0;
  } else if (rxByte < 32 || rxByte > 126) {
    rx_discard = 1;
  } else if (cmdIndex < sizeof(cmdBuf) - 1U) {
    cmdBuf[cmdIndex++] = rxByte;
  } else rx_discard = 1; /* never execute a truncated movement command */
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}


void Servo_SetAngle(uint16_t angle)
{
  if (angle > 180) angle = 180;
  uint16_t pulse = 1000 + ((uint32_t)angle * 1000) / 180;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse);
}

/* Raw servo pulse in microseconds (TIM8 ticks at 1 MHz, 20 ms period).
   Servo_SetAngle(0..180) == 1000..2000 us. Clamped to 500..2500 us. */
#define SERVO_US_MIN  500
#define SERVO_US_MAX  2500
void Servo_SetPulse(uint16_t us)
{
  if (us < SERVO_US_MIN) us = SERVO_US_MIN;
  if (us > SERVO_US_MAX) us = SERVO_US_MAX;
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, us);
}

/* Straight-ahead pulse. 1516 = Servo_SetAngle(93), so nothing changes until KC is used.
   After tuning with KC, put the chosen value here. */
static uint16_t center_us = 1490;   /* 7 Oct 2026: tuned with KC on FW100, HE -3 / +5 */
/* Steering flick before every FW/BW (takes up linkage slack).
   1000 = full left (original), 2000 = full right, 0 = no flick. */
static uint16_t flick_us = 1000;

/* HEADING HOLD during FW/BW (7 Oct 2026)
   Steers during straight moves to keep the car on its line.
   Each 10 ms: heading (gyro) and sideways offset (encoders x heading) are
   updated, and the servo is set to  centre + correction.
     error      = heading + dir * HOLD_K_LAT * sideways      (deg, + = left)
     correction = dir * HOLD_K_HEAD * error                  (us, capped)
   It starts from heading_err, so a turn that ended slightly off is also
   straightened out during the next FW/BW. heading_err is then set to what
   is left at the end. 0 = old behaviour (measure only, no steering). */
#define HOLD_ENABLE      1
#define HOLD_K_HEAD      20      /* us of steering per degree of heading error */
#define HOLD_K_LAT_X10   15      /* degrees of heading aim per cm off the line, x10 (1.5) */
#define HOLD_MAX_US      80      /* largest steering correction */
#define HOLD_START_MAX   5.0f    /* never take more than this much heading_err into one move */
static int32_t hold_en      = HOLD_ENABLE;
static int32_t hold_k_head  = HOLD_K_HEAD;
static int32_t hold_k_lat10 = HOLD_K_LAT_X10;
void Servo_Center(void) { Servo_SetPulse(center_us); }

#define PWM_MAX 4199

#define CPC_A_x100   7520L    /* 7 Oct 2026: hand push x3 + FW100 tape agree, wheels count equally */
#define CPC_B_x100   7520L
// speed: -4199 (full reverse) to +4199 (full forward), 0 = coast
void MotorA_Set(int32_t speed)
{
  if (stop_requested) speed = 0;
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
  if (stop_requested) speed = 0;
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

/* NEW: acceleration slew; original cruise/minimum PWM are unchanged.
 * VS<pwm> sets straight cruise in RAM (SPEED_MIN..PWM_MAX).
 * Enter a tested permanent cruise at SPEED_CRUISE above. */
#define TASK2_DRIVE_RAMP_MS 200U
static int32_t drive_cruise_pwm = SPEED_CRUISE;
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

void Motor_Brake(void);   /* defined further down, used to stop FW/BW */

/* ===== IE: drive forward watching ONE side IR for an obstacle edge (Task 2) =====
 * IE<side><mode><max_cm>    e.g. IERN150, IELA200
 *   side : L / R   which side IR to watch (IE_LEFT_IR / IE_RIGHT_IR below)
 *   mode : N = the obstacle must be seen (near) first, then the edge is near -> far
 *          A = any far reading counts at once (used when the robot may already be past it)
 *   max  : drive at most this far (cm), normal heading hold, like FW<max>
 * The robot stops as soon as the IR has read "far" IE_SPIKE_N samples in a row,
 * or at max_cm, or if the front ultrasonic reads below IE_WALL_GUARD_MM.
 * Reply (after the usual BRAKE line):
 *   DONE IERN150 END:<EDGE|NEVER|MAX|WALL> EDGE:<mm> NEAR:<mm> NN:<n> MOVED:<mm> HE:<deg x10> STATUS:<..>
 *     END   EDGE = near then far, NEVER = far from the start (mode A only),
 *           MAX = reached max_cm, WALL = ultrasonic guard
 *     EDGE  travel (mm) at the FIRST far sample of the run that stopped it, -1 = none
 *     NEAR  median IR distance (mm) of the near samples, -1 = none; NN = how many
 *     MOVED total travel (mm) including braking
 * The Pi (task2.py) uses EDGE / NEAR / MOVED exactly like its old FW3 + IR stepping,
 * but in one command, so there is no stop-start every 3 cm. */
#define IE_LEFT_IR        2      /* IR on the LEFT side (1 or 2) - match IR_LEFT in rpi_client_task2.py */
#define IE_RIGHT_IR       1      /* IR on the RIGHT side */
#define IE_NEAR_MAX_MM    400    /* reading <= this = obstacle beside me (task2 ir_near_max 40 cm) */
#define IE_SPIKE_N        2      /* far samples in a row that confirm the edge (task2 spike_n) */
#define IE_SAMPLE_MS      20     /* IR sample period; the Sharp sensor updates about every 38 ms */
#define IE_WALL_GUARD_MM  150    /* stop if the ultrasonic reads less (task2 wall_guard_cm); 0 = off */
#define IE_US_PERIOD_MS   100    /* ultrasonic guard ping period (a ping blocks up to ~30 ms) */
#define IE_MAX_NEAR       96

static struct {
  uint8_t  on, which, need_seen, seen, far_run, end;  /* end: 0 MAX, 1 EDGE, 2 NEVER, 3 WALL */
  uint32_t last_ir, last_us;
  int32_t  first_far_mm, edge_mm;
  int32_t  near_mm[IE_MAX_NEAR];
  int      n_near;
} ie;

static uint32_t IR_ReadRaw(uint32_t channel);
static int32_t  IR_RawTo_mm(uint32_t adc);
int32_t US_Read_mm(void);

static int32_t IE_Travel_mm(void)
{
  float cm = 0.5f * ((float)travA * 100.0f / (float)CPC_A_x100 +
                     (float)travB * 100.0f / (float)CPC_B_x100);
  return (int32_t)(cm * 10.0f);
}

/* Called every Drive_Distance loop while IE is running. Returns 1 = stop now. */
static int IE_Check(void)
{
  if (!ie.on) return 0;
  uint32_t now = HAL_GetTick();
  if (IE_WALL_GUARD_MM > 0 && now - ie.last_us >= IE_US_PERIOD_MS) {
    ie.last_us = now;
    int32_t d = US_Read_mm();
    if (d > 20 && d < IE_WALL_GUARD_MM) { ie.end = 3; return 1; }
    now = HAL_GetTick();
  }
  if (now - ie.last_ir < IE_SAMPLE_MS) return 0;
  ie.last_ir = now;

  uint32_t ch = (ie.which == 1) ? ADC_CHANNEL_11 : ADC_CHANNEL_12;
  uint32_t v[3] = { IR_ReadRaw(ch), IR_ReadRaw(ch), IR_ReadRaw(ch) };
  for (int i = 1; i < 3; i++) {                     /* median of 3 quick reads */
    uint32_t k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  int32_t mm = IR_RawTo_mm(v[1]);                   /* -1 = out of range = far */

  if (mm > 0 && mm <= IE_NEAR_MAX_MM) {
    ie.seen = 1; ie.far_run = 0;
    if (ie.n_near < IE_MAX_NEAR) ie.near_mm[ie.n_near++] = mm;
  } else if (ie.seen || !ie.need_seen) {
    if (ie.far_run == 0) ie.first_far_mm = IE_Travel_mm();
    if (++ie.far_run >= IE_SPIKE_N) {
      ie.edge_mm = ie.first_far_mm;
      ie.end = ie.seen ? 1 : 2;
      return 1;
    }
  }
  return 0;
}

/* Drive straight for dist_cm. dir: +1 = FW, -1 = BW.
   Returns 0 = arrived, 1 = timeout, 2 = stalled. */
int Drive_Distance(int32_t dist_cm, int dir)
{
  motion_status = MOTION_OK;
  travA = 0; travB = 0;
  if (dist_cm <= 0 || dist_cm > 1000 || (dir != 1 && dir != -1)) {
    motion_status = MOTION_BADARG; return 6;
  }
  if (stop_requested) { motion_status = MOTION_STOPPED; return 3; }
  if (!imu_ok || !gyro_ready) { motion_status = MOTION_IMU; return 4; }
  uint16_t move_errs = imu_err_count, move_sats = imu_sat_count;

  int32_t tgtA = (dist_cm * CPC_A_x100) / 100;
  int32_t tgtB = (dist_cm * CPC_B_x100) / 100;
  uint32_t elapsed = 0;
  uint32_t stalled = 0;
  int32_t  lastProgress = 0;
  int reason = 0;                      /* 0 arrived, 1 timeout, 2 stall */

  int32_t decel = DECEL_COUNTS;
  if (decel > (tgtA + tgtB) / 4) decel = (tgtA + tgtB) / 4;
  if (decel < 60) decel = 60;

  if (flick_us) { Servo_SetPulse(flick_us); osDelay(200); }
  Servo_Center();
  osDelay(250);
  Enc_Reset();

  float yaw = 0.0f;                    /* heading change during this move, + = left */
  uint32_t t_old = HAL_GetTick();

  /* heading hold state */
  float th0 = heading_err;             /* heading at the start vs where it should be */
  if (th0 >  HOLD_START_MAX) th0 =  HOLD_START_MAX;
  if (th0 < -HOLD_START_MAX) th0 = -HOLD_START_MAX;
  float th_extra = heading_err - th0;  /* any part not handled in this move */
  float th   = th0;                    /* heading, deg, + = left (no deadband) */
  float lat  = 0.0f;                   /* sideways offset from the start line, cm, + = left */
  float prog_old = 0.0f;               /* distance so far, cm */
  int32_t hold_max_seen = 0;           /* largest correction used, us (for the reply) */

  for (;;)
  {
    Enc_Update(dir);
    if (stop_requested) { reason = 3; break; }

    /* gyro: yaw (with deadband, old measurement) and th (for steering) */
    {
      float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
      uint32_t t_now = HAL_GetTick();
      float dt = (t_now - t_old) * 0.001f;
      t_old = t_now;
      gz *= (float)GYRO_Z_SIGN_LEFT;
      th += gz * GYRO_SCALE * dt;
      if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
      yaw += gz * dt;
    }

    if (imu_err_count != move_errs || imu_sat_count != move_sats) { reason = 4; break; }
    /* sideways offset: distance moved x sin(heading) */
    {
      float prog = 0.5f * ((float)travA * 100.0f / (float)CPC_A_x100 +
                           (float)travB * 100.0f / (float)CPC_B_x100);
      float ds = (prog - prog_old) * (float)dir;      /* + = forward */
      prog_old = prog;
      lat += ds * sinf(th * 0.0174533f);
    }

    /* steer back onto the line */
    if (hold_en)
    {
      float err  = th + (float)dir * ((float)hold_k_lat10 * 0.1f) * lat;
      int32_t c  = (int32_t)((float)dir * (float)hold_k_head * err);
      if (c >  HOLD_MAX_US) c =  HOLD_MAX_US;
      if (c < -HOLD_MAX_US) c = -HOLD_MAX_US;
      int32_t a = (c < 0) ? -c : c;
      if (a > hold_max_seen) hold_max_seen = a;
      Servo_SetPulse((uint16_t)((int32_t)center_us + c));
    }

    if (IE_Check()) break;              /* IE command: edge found / wall guard */
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

    int32_t sp = drive_cruise_pwm;
    if (rem < decel) {
      sp = SPEED_MIN + ((drive_cruise_pwm - SPEED_MIN) * rem) / decel;
    }
    if (elapsed < TASK2_DRIVE_RAMP_MS) {
      int32_t ramp = SPEED_MIN + (int32_t)((int64_t)(drive_cruise_pwm - SPEED_MIN) * elapsed / TASK2_DRIVE_RAMP_MS);
      if (sp > ramp) sp = ramp;
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
  int32_t brakeA = travA;
  int32_t brakeB = travB;
  /* 7 Oct 2026: brake instead of coasting. The coast was 0.4-2.2 cm
     depending on battery; braking makes the stop repeatable (same as turns). */
  Motor_Brake();

  /* keep measuring while it rolls to a stop */
  for (int i = 0; i < 10; i++)
  {
	Enc_Update(dir);
    float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
    uint32_t t_now = HAL_GetTick();
    float dt = (t_now - t_old) * 0.001f;
    t_old = t_now;
    gz *= (float)GYRO_Z_SIGN_LEFT;
    th += gz * GYRO_SCALE * dt;
    if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
    yaw += gz * dt;
    {
      float prog = 0.5f * ((float)travA * 100.0f / (float)CPC_A_x100 +
                           (float)travB * 100.0f / (float)CPC_B_x100);
      float ds = (prog - prog_old) * (float)dir;
      prog_old = prog;
      lat += ds * sinf(th * 0.0174533f);
    }
    osDelay(20);
  }

  MotorA_Set(0);                       /* release the brake */
  MotorB_Set(0);

  /* steering stays where it was during the roll-out (no end-of-move twitch);
     re-centre only once the car has stopped */
  if (hold_en) Servo_Center();

  if (hold_en)
    heading_err = th + th_extra;       /* what is really left after this move */
  else
    heading_err += yaw * GYRO_SCALE;   /* old behaviour */
  if (heading_err >  HEADING_ERR_MAX) heading_err =  HEADING_ERR_MAX;
  if (heading_err < -HEADING_ERR_MAX) heading_err = -HEADING_ERR_MAX;

  {
    char t[96];
    int n = sprintf(t, "BRAKE A:%ld B:%ld  FINAL A:%ld B:%ld  Y:%ld HC:%ld\r\n",
                    brakeA, brakeB, travA, travB,
                    (int32_t)(lat * 10.0f), hold_max_seen);
    HAL_UART_Transmit(&huart3, (uint8_t *)t, n, 100);
  }

  if (stop_requested) reason = MOTION_STOPPED;
  else if (imu_err_count != move_errs || imu_sat_count != move_sats) reason = MOTION_IMU;
  motion_status = reason;
  return reason;
}

/* ================= Turning ================= */
#define TURN_LOOP_MS    20
#define TURN_TIMEOUT_MS 5000
#define TURN_PWM_OUTER  4100
/* Inner rear wheel PWM during a turn. Original: outer/1.71 (matched to
   steering geometry). Lower = tighter, but with tyre scrub.
   NOTE: changing this changes turn rate -> recalibrate STOP_OFFSET_*. */
#define TURN_PWM_INNER  ((int32_t)(TURN_PWM_OUTER / 1.71f))   /* ~2397 */

/* Turn speed per turn type, as % of normal turn PWM (both rear wheels).
   Lower = less skidding, but the car coasts less, so that turn's
   STOP_OFFSET_* must be recalibrated after changing it. */
/* Inner rear wheel PWM per turn type (before the speed % above).
   Lower = tighter arc (with some tyre scrub). Default = outer/1.71. */
#define TURN_INNER_LT   TURN_PWM_INNER
#define TURN_INNER_RT   TURN_PWM_INNER  /* 2 Oct 2026: 2000 tightened RT at full speed (34 -> 30 cm)
                                           by extra scrub; back to 2397 now RT runs at 75 % for consistency */
#define TURN_INNER_XL   2550
#define TURN_INNER_XR   TURN_PWM_INNER

#define TURN_SPEED_PCT_LT   100
#define TURN_SPEED_PCT_RT   100     /* 3 Oct 2026: 100 skids. 75 is steady but wider (~40 x 24 at KR2030) */
#define TURN_SPEED_PCT_XL   75     /* 2 Oct 2026: 100 skidded (MAX 120-209), 75 gives MAX ~60 steady */
#define TURN_SPEED_PCT_XR   75     /* 7 Oct 2026: 100 skids on XR; 75 is steady (XL stays 100) */     /* 2 Oct 2026: 100 uneven turn to turn, 75 gives MAX ~50 steady */

/* Square-turn compensation (cm), for 90-degree turns only.
   The planner tracks the CENTRE of the car, which sits ahead of the rear
   axle, so a plain arc moves it further one way than the other.
   LT/RT: reverse this far AFTER the turn  (shrinks the sideways distance).
   XL/XR: drive forward this far BEFORE the turn (shrinks the lengthwise distance).
   Set each to (bigger side - smaller side) measured without compensation.
   2 Oct 2026, raw boxes (sideways x lengthwise, move off) at final settings:
   LT 28x15, RT 31x18 (inner 2000), XL 16x33 (75 %), XR 26x38 (75 %). */
/* DISABLED (0 = no extra move). Measured values if you re-enable:
   LT 13, RT 13, XL 17, XR 12. */
#define TURN_COMP_LT_CM   0
#define TURN_COMP_RT_CM   0
#define TURN_COMP_XL_CM   0
#define TURN_COMP_XR_CM   0

/* SPEED-ADAPTIVE STOP
   The car keeps rotating after it brakes, and how far depends on how fast
   it was spinning (battery level changes that from session to session).
   With ADAPTIVE_STOP = 1 a turn brakes when
       angle_so_far + COAST_K * spin_rate  >=  angle_wanted
   so a fast spin brakes earlier and a slow one later, automatically.
   COAST_K is in seconds (= degrees of coast per deg/s of spin), per turn type.
   Calibrate: run the turn after BI; the reply gives IMU (angle) and R (spin rate
   at braking). new K = old K + (IMU/10 - target) / R
   (overshoot -> K up, brakes earlier; undershoot -> K down, brakes later).
   0 = old fixed STOP_OFFSET_* in degrees. */
#define ADAPTIVE_STOP     1
#define COAST_K_LT        0.104f
#define COAST_K_RT        0.123f
#define COAST_K_XL        0.065f  /* from: 4.2 at ~62 deg/s (75 %) */
#define COAST_K_XR        0.092f  /* tuned at 75 % */  /* from: 4.8 at ~52 deg/s (75 %) */

/* What the steering does while the car coasts after braking a turn.
   0 = old behaviour: always swing to SERVO_LEFT (counter-steers on RT and XR!)
   1 = hold the turn's own lock, same for every turn (symmetric). */
#define COAST_HOLD_LOCK   0   /* 0 and 1 gave the same RT box; offsets were measured with 0 */

/* Steering lock pulses used by Car_Turn. Original = 1000 / 2000 us
   (same as Servo_SetAngle(SERVO_LEFT / SERVO_RIGHT)).
   Chosen from SU sweep: left buzzes at 905, right strains at 2050. */
#define STEER_LEFT_US_DEFAULT   950
#define STEER_RIGHT_US_DEFAULT  2000   /* 3 Oct 2026: 2030 strained the servo on the floor;
                                                RT went inconsistent after a while. 2000 stays consistent. */

/* POST-TURN TRIM (Jinglin's idea, 3 Oct 2026)
   After the turn has braked and coasted, compare the measured angle with the
   angle the turn aimed for. If it is more than TRIM_THRESH_DEG off, re-apply the
   turn's steering lock and creep ONCE:
     short     -> creep the same way the turn went (forward for LT/RT, back for XL/XR)
     overshot  -> creep the opposite way
   With the wheels locked, a creep is a small turn, so the angle moves toward the
   aim. It stops on the gyro, and is capped at TRIM_MAX_DEG and TRIM_TIMEOUT_MS.
   Whatever error is left still goes into heading_err for the next turn.
   TRIM_PWM must be above the ~2400 start-from-rest threshold. */
#define TRIM_ENABLE        1
#define TRIM_PWM           2800    /* outer wheel; inner = TRIM_PWM / 1.71 like a turn */
#define TRIM_THRESH_DEG    1.0f    /* only trim if further off than this */
#define TRIM_MAX_DEG       8.0f    /* never try to fix more than this */
#define TRIM_LEAD_DEG      0.5f    /* brake this early (slow creep coasts very little) */
#define TRIM_TIMEOUT_MS    1500

/* ===== STEERING TEST MODE =====================================
   1 = enable runtime commands SU / KL / KR / KC / IP / KQ / KS / KH / KA / KT / KM / T / V / N / C for testing.
   0 = remove them; turns use the #defines above.
   After testing: copy your chosen values into the #defines above,
   then set this to 0 (or delete every "#if STEER_TEST" block).   */
#define STEER_TEST  1

#if STEER_TEST
static uint16_t steer_left_us  = STEER_LEFT_US_DEFAULT;
static uint16_t steer_right_us = STEER_RIGHT_US_DEFAULT;
static int32_t  turn_inner[4]  = { TURN_INNER_LT, TURN_INNER_RT,
                                   TURN_INNER_XL, TURN_INNER_XR };
static int32_t  turn_pct[4]    = { TURN_SPEED_PCT_LT, TURN_SPEED_PCT_RT,
                                   TURN_SPEED_PCT_XL, TURN_SPEED_PCT_XR };
#define STEER_LEFT_US    steer_left_us
#define STEER_RIGHT_US   steer_right_us
#define TURN_INNER_NOW(i) turn_inner[i]
#define TURN_PCT_NOW(i)  turn_pct[i]
static int32_t  coast_hold     = COAST_HOLD_LOCK;
#define COAST_HOLD_NOW   coast_hold
static int32_t  adaptive_stop  = ADAPTIVE_STOP;
#define ADAPTIVE_NOW     adaptive_stop
static float    coast_k[4]     = { COAST_K_LT, COAST_K_RT, COAST_K_XL, COAST_K_XR };
#define COAST_K_NOW(i)   coast_k[i]
static int32_t  turn_comp[4]   = { TURN_COMP_LT_CM, TURN_COMP_RT_CM,
                                   TURN_COMP_XL_CM, TURN_COMP_XR_CM };
#define TURN_COMP_NOW(i) turn_comp[i]
static int32_t  trim_enable    = TRIM_ENABLE;
#define TRIM_EN_NOW      trim_enable
static int32_t  trim_pwm       = TRIM_PWM;
#define TRIM_PWM_NOW     trim_pwm
#else
#define STEER_LEFT_US    STEER_LEFT_US_DEFAULT
#define STEER_RIGHT_US   STEER_RIGHT_US_DEFAULT
static const int32_t turn_inner_fixed[4] = { TURN_INNER_LT, TURN_INNER_RT,
                                             TURN_INNER_XL, TURN_INNER_XR };
#define TURN_INNER_NOW(i) turn_inner_fixed[i]
static const int32_t turn_pct_fixed[4] = { TURN_SPEED_PCT_LT, TURN_SPEED_PCT_RT,
                                           TURN_SPEED_PCT_XL, TURN_SPEED_PCT_XR };
#define TURN_PCT_NOW(i)  turn_pct_fixed[i]
#define COAST_HOLD_NOW   COAST_HOLD_LOCK
#define ADAPTIVE_NOW     ADAPTIVE_STOP
static const float coast_k_fixed[4] = { COAST_K_LT, COAST_K_RT, COAST_K_XL, COAST_K_XR };
#define COAST_K_NOW(i)   coast_k_fixed[i]
static const int32_t turn_comp_fixed[4] = { TURN_COMP_LT_CM, TURN_COMP_RT_CM,
                                            TURN_COMP_XL_CM, TURN_COMP_XR_CM };
#define TURN_COMP_NOW(i) turn_comp_fixed[i]
#define TRIM_EN_NOW      TRIM_ENABLE
#define TRIM_PWM_NOW     TRIM_PWM
#endif
/* ============================================================== */


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
    osDelay(60); /* leave time between ultrasonic triggers */
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

/* ---------- Sharp GP2Y0A21YK0F : IR1 = PC1 (ch11), IR2 = PC2 (ch12) ---------- */
static uint32_t IR_ReadRaw(uint32_t channel)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = channel;
  s.Rank = 1;
  s.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) return 0;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint32_t v = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;                                   /* 0..4095 */
}

/* median of 5 raw readings; which = 1 or 2 */
uint32_t IR_ReadADC(int which)
{
  uint32_t ch = (which == 1) ? ADC_CHANNEL_11 : ADC_CHANNEL_12;
  uint32_t v[5];
  for (int i = 0; i < 5; i++) { v[i] = IR_ReadRaw(ch); osDelay(5); }
  for (int i = 1; i < 5; i++) {
    uint32_t k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[2];
}

/* CALIBRATE THESE from your own readings (see below) */
#define IR_K       30088.0f
#define IR_OFFSET  133.0f

/* distance in mm, or -1 if out of the sensor's usable range */
static int32_t IR_RawTo_mm(uint32_t adc)
{
  float raw = (float)adc;
  if (raw <= IR_OFFSET + 1.0f) return -1;
  float cm = IR_K / (raw - IR_OFFSET);
  if (cm < 8.0f || cm > 80.0f) return -1;
  return (int32_t)(cm * 10.0f);
}

int32_t IR_Read_mm(int which)
{
  if (which != 1 && which != 2) return -1;
  return IR_RawTo_mm(IR_ReadADC(which));
}

/* Gyro bias calibration with a stillness check.
   If the car moves during the 1 s measurement (someone touches it, the table
   is bumped), the readings spread out and the bias comes out wrong, which then
   skews every turn. So: measure, check the spread, retry up to 3 times.
   bias_status: 0 = OK first try, n = OK after n retries,
                -1 = never still: kept the previous bias (or the steadiest try). */
#define BIAS_MAX_STD_DPS   0.25f   /* noise when still is ~0.05-0.1 dps */
#define BIAS_MAX_P2P_DPS   1.5f    /* peak-to-peak; catches a single bump */
static int bias_status = 0;

void Gyro_CalibrateZBias(void)
{
  gyro_ready = 0;
  if (!imu_ok) { bias_status = -1; return; }
  float prev = gyro_bias_z;
  int   have_prev = (prev != 0.0f);
  float best_mean = 0.0f, best_std = 1e9f;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint16_t errors_before = imu_err_count, sats_before = imu_sat_count;
    float sum = 0.0f, sum2 = 0.0f, lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 200; i++) {
      float g = IMU_GetGyroZ_Raw();
      sum += g; sum2 += g * g;
      if (g < lo) lo = g;
      if (g > hi) hi = g;
      osDelay(5);
    }
    float mean = sum / 200.0f;
    float var  = sum2 / 200.0f - mean * mean;
    float sd   = (var > 0.0f) ? sqrtf(var) : 0.0f;

    if (sd < best_std) { best_std = sd; best_mean = mean; }

    if (!stop_requested && imu_err_count == errors_before && imu_sat_count == sats_before &&
        sd <= BIAS_MAX_STD_DPS && (hi - lo) <= BIAS_MAX_P2P_DPS) {
      gyro_ready = 1;
      gyro_bias_z = mean;
      bias_status = attempt;
      return;
    }
    osDelay(200);   /* give whatever moved it a moment to settle */
  }

  /* never still enough */
  gyro_bias_z = have_prev ? prev : best_mean;
  bias_status = -1;
}

/* Reply text for BI and start-up: "BIAS -443 (x1000)" + note if not clean */
static int Bias_Report(char *tx)
{
  if (bias_status == 0)
    return sprintf(tx, "BIAS %ld (x1000)\r\n", (int32_t)(gyro_bias_z * 1000));
  if (bias_status > 0)
    return sprintf(tx, "BIAS %ld (x1000) RETRIED:%d\r\n", (int32_t)(gyro_bias_z * 1000), bias_status);
  return sprintf(tx, "BIAS %ld (x1000) UNSTABLE KEEP STILL AND SEND BI AGAIN\r\n", (int32_t)(gyro_bias_z * 1000));
}

/* Integrate gyro Z into *angle for `ms` milliseconds (10 ms steps).
   sgn makes the turn direction positive. Used while waiting for the
   servo so rotation during those waits is not lost. */
static void Turn_Integrate(float *angle, uint32_t *t_old, float sgn, uint32_t ms)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < ms)
  {
    uint16_t errs = imu_err_count, sats = imu_sat_count;
    float gz = IMU_GetGyroZ_Raw() - gyro_bias_z;
    if (imu_err_count != errs || imu_sat_count != sats) {
      motion_status = MOTION_IMU; return;
    }
    if (stop_requested) motion_status = MOTION_STOPPED;
    uint32_t t_now = HAL_GetTick();
    float dt = (t_now - *t_old) * 0.001f;
    *t_old = t_now;
    gz *= sgn;
    if (gz > -GYRO_DEADBAND && gz < GYRO_DEADBAND) gz = 0.0f;
    *angle += gz * dt;
    osDelay(10);
  }
}

/* Friend's useful mechanisms: remaining-angle speed stages, stationary inner
 * motor and measurement through the final stop. Uses OUR motor/gyro/servo API.
 * New settings below are starting points, NOT measured robot calibration.
 * TM1 = new single-wheel mechanism (default); TM0 = original wheel ratio.
 * The original N/IP, KH, KA, T and V tuning commands remain available.
 * In TM1, N/IP and KH do not affect the turn: inner PWM is zero and steering
 * stays locked through braking. In TM0 those original settings are used.
 */
#define TASK2_TURN_SINGLE_WHEEL  1
#define TASK2_TURN_RAMP_MS       180U
#define TASK2_TURN_SETTLE_MS     150U
#define TASK2_TURN_LEAD_CAP_DEG  3.0f
#define TASK2_TURN_TOL_DEG       2.0f
static int turn_single_wheel = TASK2_TURN_SINGLE_WHEEL;

static int32_t Turn_SpeedForRemaining(float remaining, int ti)
{
  int32_t maximum = TURN_PWM_OUTER * TURN_PCT_NOW(ti) / 100;
  /* Our backup says approximately 2400 PWM is needed to start turning.
   * Keep its 2800 trim PWM as the starting floor, not friend's 1000/7199. */
  int32_t floor = TRIM_PWM_NOW;
  if (floor > maximum) floor = maximum;
  if (remaining <= 10.0f) return floor;
  if (remaining < 30.0f) return floor + (maximum - floor) / 3;
  if (remaining < 45.0f) return floor + 2 * (maximum - floor) / 3;
  if (remaining < 60.0f) return floor + 5 * (maximum - floor) / 6;
  return maximum;
}

static void Turn_SetMotors(int32_t pwm, int direction, int drive_dir, int ti)
{
  int32_t inner = turn_single_wheel ? 0 :
                  (int32_t)((int64_t)pwm * TURN_INNER_NOW(ti) / TURN_PWM_OUTER);
  if (direction == TURN_LEFT) {
    MotorA_Set(drive_dir * inner); MotorB_Set(drive_dir * pwm);
  } else {
    MotorA_Set(drive_dir * pwm); MotorB_Set(drive_dir * inner);
  }
}

float Car_Turn(float target_deg, int direction, int drive_dir)
{
  motion_status = MOTION_OK;
  turn_trim = 0.0f;
  turn_brake_rate = 0.0f;
  if (!imu_ok || !gyro_ready) { motion_status = MOTION_IMU; return 0.0f; }
  if (stop_requested) { motion_status = MOTION_STOPPED; return 0.0f; }
  if (!(target_deg > 0.0f && target_deg <= 360.0f) ||
      (direction != 1 && direction != -1) || (drive_dir != 1 && drive_dir != -1)) {
    motion_status = MOTION_BADARG; return 0.0f;
  }
  float angle = 0.0f; /* raw gyro degrees; convert at EVERY comparison */
  float rot = (float)(direction * drive_dir);
  float carry = rot * heading_err;
  float carry_limit = fminf(HOLD_START_MAX, target_deg * 0.25f);
  if (carry > carry_limit) carry = carry_limit;
  if (carry < -carry_limit) carry = -carry_limit;
  float want = target_deg - carry;
  turn_want = want;
  int ti = (drive_dir < 0 ? 2 : 0) + (direction == TURN_LEFT ? 0 : 1);
  float sgn = (float)GYRO_Z_SIGN_LEFT * rot;
  float offset = drive_dir < 0 ?
    (direction == TURN_LEFT ? STOP_OFFSET_REV_LEFT : STOP_OFFSET_REV_RIGHT) :
    (target_deg < SMALL_TURN_THRESHOLD ?
     (direction == TURN_LEFT ? STOP_OFFSET_SMALL_LEFT : STOP_OFFSET_SMALL_RIGHT) :
     (direction == TURN_LEFT ? STOP_OFFSET_LEFT : STOP_OFFSET_RIGHT));
  float rate_f = 0.0f;
  imu_err_count = 0; imu_sat_count = 0; imu_max_dps = 0.0f;
  uint32_t t_old = HAL_GetTick();
  Servo_SetPulse(direction == TURN_LEFT ? STEER_LEFT_US : STEER_RIGHT_US);
  Turn_Integrate(&angle, &t_old, sgn, 250);
  uint32_t start = HAL_GetTick();
  uint32_t progress_tick = start;
  float progress_angle = angle;

  while (motion_status == MOTION_OK)
  {
    if (stop_requested) { motion_status = MOTION_STOPPED; break; }
    uint16_t errs = imu_err_count, sats = imu_sat_count;
    float gz = (IMU_GetGyroZ_Raw() - gyro_bias_z) * sgn;
    uint32_t now = HAL_GetTick();
    if (imu_err_count != errs || imu_sat_count != sats) {
      motion_status = MOTION_IMU; break;
    }
    float dt = (now - t_old) * 0.001f;
    t_old = now;
    if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;
    angle += gz * dt;
    rate_f = 0.7f * rate_f + 0.3f * gz;
    float remaining = want - angle * GYRO_SCALE;
    /* Preserve group's coast constants, but cap their lead for a small turn.
     * Old 14-16 degree offsets must not cancel a requested 10 degree turn. */
    float lead = ADAPTIVE_NOW ?
      COAST_K_NOW(ti) * fmaxf(rate_f, 0.0f) * GYRO_SCALE : offset;
    float cap = fminf(TASK2_TURN_LEAD_CAP_DEG, want * 0.20f);
    lead = fmaxf(0.0f, fminf(lead, cap));
    if (remaining <= lead) break;
    if (now - start >= TURN_TIMEOUT_MS) { motion_status = MOTION_TIMEOUT; break; }
    if ((angle - progress_angle) * GYRO_SCALE >= 0.5f) {
      progress_tick = now; progress_angle = angle;
    } else if (now - progress_tick >= 700U) {
      motion_status = MOTION_STALL; break;
    }
    int32_t pwm = Turn_SpeedForRemaining(remaining, ti);
    int32_t floor = Turn_SpeedForRemaining(0.0f, ti);
    uint32_t elapsed = now - start;
    if (elapsed < TASK2_TURN_RAMP_MS)
      pwm = floor + (int32_t)((int64_t)(pwm - floor) * elapsed / TASK2_TURN_RAMP_MS);
    Turn_SetMotors(pwm, direction, drive_dir, ti);
    osDelay(10);
  }

  turn_brake_rate = rate_f * GYRO_SCALE;
  Motor_Brake();
  if (!turn_single_wheel && !COAST_HOLD_NOW) Servo_SetAngle(SERVO_LEFT);
  Turn_Integrate(&angle, &t_old, sgn, TASK2_TURN_SETTLE_MS);

  /* Retain one bounded correction from our backup; use the SAME wheel
   * mechanism as the main turn, and never trim after a fault or STOP. */
  float err = want - angle * GYRO_SCALE;
  if (motion_status == MOTION_OK && !stop_requested && TRIM_EN_NOW &&
      fabsf(err) > TRIM_THRESH_DEG && fabsf(err) <= TRIM_MAX_DEG)
  {
    float before = angle;
    float trim_sign = err > 0.0f ? 1.0f : -1.0f;
    Servo_SetPulse(direction == TURN_LEFT ? STEER_LEFT_US : STEER_RIGHT_US);
    Turn_Integrate(&angle, &t_old, sgn, 100);
    uint32_t trim_start = HAL_GetTick();
    while (motion_status == MOTION_OK && !stop_requested)
    {
      uint16_t errs = imu_err_count, sats = imu_sat_count;
      float gz = (IMU_GetGyroZ_Raw() - gyro_bias_z) * sgn;
      uint32_t now = HAL_GetTick();
      if (imu_err_count != errs || imu_sat_count != sats) {
        motion_status = MOTION_IMU; break;
      }
      angle += gz * (now - t_old) * 0.001f;
      t_old = now;
      if (trim_sign * (want - angle * GYRO_SCALE) <= TRIM_LEAD_DEG) break;
      if (now - trim_start >= TRIM_TIMEOUT_MS) { motion_status = MOTION_TIMEOUT; break; }
      Turn_SetMotors(Turn_SpeedForRemaining(0.0f, ti), direction,
                     trim_sign > 0.0f ? drive_dir : -drive_dir, ti);
      osDelay(10);
    }
    Motor_Brake();
    Turn_Integrate(&angle, &t_old, sgn, TASK2_TURN_SETTLE_MS);
    turn_trim = (angle - before) * GYRO_SCALE;
  }
  Servo_Center();
  Turn_Integrate(&angle, &t_old, sgn, TASK2_TURN_SETTLE_MS);
  MotorA_Set(0); MotorB_Set(0);
  float actual = angle * GYRO_SCALE;
  if (stop_requested) motion_status = MOTION_STOPPED;
  if (motion_status == MOTION_OK && fabsf(want - actual) > TASK2_TURN_TOL_DEG)
    motion_status = MOTION_ANGLE;
  if (motion_status == MOTION_OK) {
    heading_err += rot * (actual - target_deg);
    heading_err = fmaxf(-HEADING_ERR_MAX, fminf(heading_err, HEADING_ERR_MAX));
  } else {
    heading_err = 0.0f; /* Pi must replan after a failed move. */
  }
  return actual;
}



/* Turn with square-turn compensation (90-degree turns only).
   Other angles are passed straight to Car_Turn unchanged. */
float Car_Turn_Square(float target_deg, int direction, int drive_dir)
{
  int ti = ((drive_dir < 0) ? 2 : 0) + ((direction == TURN_LEFT) ? 0 : 1);
  int32_t comp = (target_deg > 89.0f && target_deg < 91.0f) ? TURN_COMP_NOW(ti) : 0;

  if (comp > 0 && drive_dir < 0) {          /* XL / XR: forward first */
    if (Drive_Distance(comp, +1) != 0) return 0.0f;
    osDelay(100);
  }

  float got = Car_Turn(target_deg, direction, drive_dir);

  if (motion_status == MOTION_OK && comp > 0 && drive_dir > 0) {          /* LT / RT: reverse after */
    osDelay(100);
    Drive_Distance(comp, -1);
  }
  return got;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    cmdIndex = 0;
    rx_discard = 1;
    rx_rejected = 1;
    HAL_UART_Receive_IT(&huart3, &rxByte, 1);
  }
}
/* AC<cm>: continuously approach a front ultrasonic target, reversing if
 * initially too close. No encoder-distance budget and no iteration limit.
 * Stops on settled +/-AC_TOL_MM, invalid echo, gyro fault, stall, STOP or
 * elapsed timeout. Test stopping distance on the actual robot before use.
 * New timing/range settings are controller defaults, not calibration data. */
#define AC_TIMEOUT_MS       60000U
#define AC_PING_PERIOD_MS   60U
#define AC_SLOW_ZONE_MM     300
#define AC_VALID_MAX_MM     4000
#define AC_SETTLE_MS        150U

int Car_ApproachUltrasonic(int32_t target_mm, int32_t *final_mm)
{
  uint32_t started = HAL_GetTick();
  float net_cm = 0.0f;
  float angle = heading_err / GYRO_SCALE;
  uint32_t gyro_tick = HAL_GetTick();
  motion_status = MOTION_OK;
  ap_moved_cm = 0;
  *final_mm = -1;
  if (target_mm < 20 || target_mm > AC_VALID_MAX_MM) {
    motion_status = MOTION_BADARG; return motion_status;
  }
  if (!imu_ok || !gyro_ready) { motion_status = MOTION_IMU; return motion_status; }
  MotorA_Set(0); MotorB_Set(0); Servo_Center();

  while (motion_status == MOTION_OK)
  {
    /* Confirm the final spacing with the robot stopped. A transient running
     * echo can make us brake, but cannot falsely declare target reached. */
    int32_t d = US_Median_mm();
    *final_mm = d;
    gyro_tick = HAL_GetTick(); /* wheels are stationary during this median */
    if (stop_requested) { motion_status = MOTION_STOPPED; break; }
    if (HAL_GetTick() - started >= AC_TIMEOUT_MS) { motion_status = MOTION_TIMEOUT; break; }
    if (d < 20 || d > AC_VALID_MAX_MM) { motion_status = MOTION_NOOBJ; break; }
    int32_t error = d - target_mm;
    if (labs(error) <= AC_TOL_MM) break;
    int dir = error > 0 ? 1 : -1;
    Enc_Reset();
    uint32_t segment_start = HAL_GetTick();
    uint32_t ping_tick = segment_start;
    uint32_t progress_tick = segment_start;
    int32_t progress_last = 0;
    float lat = 0.0f, old_cm = 0.0f;

    while (motion_status == MOTION_OK)
    {
      uint32_t now = HAL_GetTick();
      if (stop_requested) { motion_status = MOTION_STOPPED; break; }
      if (now - started >= AC_TIMEOUT_MS) { motion_status = MOTION_TIMEOUT; break; }
      Enc_Update(dir);
      int32_t progress = (travA + travB) / 2;
      if (progress - progress_last >= STALL_COUNTS) {
        progress_last = progress; progress_tick = now;
      } else if (now - progress_tick >= STALL_MS) {
        motion_status = MOTION_STALL; break;
      }
      uint16_t errors = imu_err_count, sats = imu_sat_count;
      float gz = (IMU_GetGyroZ_Raw() - gyro_bias_z) * GYRO_Z_SIGN_LEFT;
      now = HAL_GetTick();
      if (imu_err_count != errors || imu_sat_count != sats) {
        motion_status = MOTION_IMU; break;
      }
      angle += gz * (now - gyro_tick) * 0.001f;
      gyro_tick = now;
      float cm = 0.5f * ((float)travA * 100.0f / CPC_A_x100 +
                         (float)travB * 100.0f / CPC_B_x100);
      lat += dir * (cm - old_cm) * sinf(angle * GYRO_SCALE * 0.0174533f);
      old_cm = cm;
      if (hold_en) {
        float err = angle * GYRO_SCALE + dir * (hold_k_lat10 * 0.1f) * lat;
        int32_t correction = (int32_t)(dir * hold_k_head * err);
        if (correction > HOLD_MAX_US) correction = HOLD_MAX_US;
        if (correction < -HOLD_MAX_US) correction = -HOLD_MAX_US;
        Servo_SetPulse((uint16_t)((int32_t)center_us + correction));
      }
      if (now - ping_tick >= AC_PING_PERIOD_MS) {
        d = US_Read_mm();
        ping_tick = HAL_GetTick();
        *final_mm = d;
        if (stop_requested) { motion_status = MOTION_STOPPED; break; }
        if (d < 20 || d > AC_VALID_MAX_MM) { motion_status = MOTION_NOOBJ; break; }
      }
      int32_t remaining = dir * (d - target_mm);
      if (remaining <= AC_TOL_MM) break; /* brake before any reversal */
      int32_t pwm = drive_cruise_pwm;
      if (remaining < AC_SLOW_ZONE_MM)
        pwm = SPEED_MIN + (int32_t)((int64_t)(drive_cruise_pwm - SPEED_MIN) *
                                   remaining / AC_SLOW_ZONE_MM);
      uint32_t elapsed = HAL_GetTick() - segment_start;
      if (elapsed < TASK2_DRIVE_RAMP_MS) {
        int32_t ramp = SPEED_MIN + (int32_t)((int64_t)(drive_cruise_pwm - SPEED_MIN) *
                                            elapsed / TASK2_DRIVE_RAMP_MS);
        if (pwm > ramp) pwm = ramp;
      }
      int32_t normB = (int32_t)((int64_t)travB * CPC_A_x100 / CPC_B_x100);
      int32_t correction = KP_SYNC * (travA - normB);
      int32_t limit = pwm / 3;
      if (limit > PWM_MAX - pwm) limit = PWM_MAX - pwm;
      if (correction > limit) correction = limit;
      if (correction < -limit) correction = -limit;
      MotorA_Set(dir * (pwm - correction));
      MotorB_Set(dir * (pwm + correction));
      osDelay(CTRL_PERIOD_MS);
    }
    Motor_Brake();
    Turn_Integrate(&angle, &gyro_tick, (float)GYRO_Z_SIGN_LEFT, AC_SETTLE_MS);
    Enc_Update(dir); /* include movement during the last sample and braking */
    net_cm += dir * 0.5f * ((float)travA * 100.0f / CPC_A_x100 +
                            (float)travB * 100.0f / CPC_B_x100);
    MotorA_Set(0); MotorB_Set(0); Servo_Center();
    /* The outer loop verifies spacing, then corrects again if necessary. */
  }
  MotorA_Set(0); MotorB_Set(0); Servo_Center();
  if (stop_requested) motion_status = MOTION_STOPPED;
  ap_moved_cm = (int32_t)roundf(net_cm);
  heading_err = fmaxf(-HEADING_ERR_MAX, fminf(angle * GYRO_SCALE, HEADING_ERR_MAX));
  return motion_status;
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
  char tx[256];

  osDelay(200);
  imu_ok = IMU_Init();
  {
    int n = sprintf(tx, "IMU %s\r\n", imu_ok ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
  }
  Gyro_CalibrateZBias();
  {
    int n = Bias_Report(tx);
    HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
  }
  Servo_Center();
  US_Init();
  for (;;)
  {
    if (stop_requested) {
      Motor_Brake(); osDelay(100);
      MotorA_Set(0); MotorB_Set(0); Servo_Center();
      heading_err = 0.0f;
      taskENTER_CRITICAL();
      rx_tail = rx_head; stop_requested = 0;
      taskEXIT_CRITICAL();
      HAL_UART_Transmit(&huart3, (uint8_t *)"DONE STOP\r\n", 11, 100);
    }
    if (rx_rejected) {
      taskENTER_CRITICAL(); rx_rejected = 0; taskEXIT_CRITICAL();
      HAL_UART_Transmit(&huart3, (uint8_t *)"ERR RX_REJECTED\r\n", 17, 100);
    }
    if (rx_tail != rx_head)
    {
      taskENTER_CRITICAL();
      memcpy(cmd, rx_queue[rx_tail], sizeof(cmd));
      rx_tail = (uint8_t)((rx_tail + 1U) % RX_QUEUE_SLOTS);
      memcpy(aRxBuffer, cmd, sizeof(cmd));
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

      size_t plen = strlen(p);
      while (plen && p[plen - 1] == ' ') p[--plen] = '\0';
      int argument_at = 2;
      if (p[0] == 'T' && p[1] != 'M') argument_at = 3;
      if (p[0] == 'I' && p[1] == 'E') argument_at = 4;   /* IE<L|R><N|A><cm> */
      if (p[0] == 'N' || p[0] == 'C' || (p[0] == 'V' && p[1] != 'S')) argument_at = 3;
      int noarg = !strcmp(p,"US") || !strcmp(p,"IR") || !strcmp(p,"BI") ||
                  !strcmp(p,"KQ") || !strcmp(p,"RA") || !strcmp(p,"GY") || !strcmp(p,"YW");
      int sensor_prefix = !strncmp(p,"US",2) || !strncmp(p,"IR",2) ||
                          !strncmp(p,"BI",2) || !strncmp(p,"RA",2) ||
                          !strncmp(p,"GY",2) || !strncmp(p,"YW",2) || !strncmp(p,"KQ",2);
      if (sensor_prefix && !noarg) {
        int n = sprintf(tx, "ERR BADARG [%s]\r\n", p);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100); continue;
      }
      if (!noarg) {
        char *end = NULL;
        errno = 0;
        long value = plen > (size_t)argument_at ? strtol(p + argument_at, &end, 10) : 0;
        if (plen <= (size_t)argument_at || end == p + argument_at || *end || errno ||
            value < -10000 || value > 10000) {
          int n = sprintf(tx, "ERR BADARG [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
          continue;
        }
      }

      int dir = 0;
      if ((p[0] == 'F' || p[0] == 'f') && (p[1] == 'W' || p[1] == 'w')) dir = +1;
      if ((p[0] == 'B' || p[0] == 'b') && (p[1] == 'W' || p[1] == 'w')) dir = -1;

      if (dir != 0)
      {
        int32_t d = atoi(&p[2]);
        if (d > 0 && d <= 1000)
        {
          Drive_Distance(d, dir);
          int n = sprintf(tx, "DONE %s A:%ld B:%ld HE:%ld %s\r\n", p, travA, travB,
                          (int32_t)(heading_err * 10),
                          Motion_Status());
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
        if (a > 0 && a <= 360)
        {
          float got = Car_Turn_Square((float)a, TURN_LEFT, +1);
          int n = sprintf(tx, "DONE %s IMU:%ld W:%ld TRIM:%ld MAX:%ld R:%ld ERR:%u SAT:%u STATUS:%s\r\n", p, (int32_t)(got * 10), (int32_t)(turn_want * 10), (int32_t)(turn_trim * 10), (int32_t)imu_max_dps, (int32_t)turn_brake_rate, imu_err_count, imu_sat_count, Motion_Status());
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
        if (a > 0 && a <= 360)
        {
          float got = Car_Turn_Square((float)a, TURN_RIGHT, +1);
          int n = sprintf(tx, "DONE %s IMU:%ld W:%ld TRIM:%ld MAX:%ld R:%ld ERR:%u SAT:%u STATUS:%s\r\n", p, (int32_t)(got * 10), (int32_t)(turn_want * 10), (int32_t)(turn_trim * 10), (int32_t)imu_max_dps, (int32_t)turn_brake_rate, imu_err_count, imu_sat_count, Motion_Status());
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
        if (a < 0 || a > 180) {
          HAL_UART_Transmit(&huart3, (uint8_t *)"ERR SV_RANGE\r\n", 14, 100); continue;
        }
        Servo_SetAngle(a);
        int n = sprintf(tx, "SERVO %d\r\n", a);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'S' || p[0] == 's') && (p[1] == 'P' || p[1] == 'p'))
      {
        int32_t v = atoi(&p[2]);
        if (v < -PWM_MAX || v > PWM_MAX) {
          HAL_UART_Transmit(&huart3, (uint8_t *)"ERR SP_RANGE\r\n", 14, 100); continue;
        }
        int sdir = (v >= 0) ? +1 : -1;
        Enc_Reset();
        MotorA_Set(v);
        MotorB_Set(v);
        for (int i = 0; i < 50 && !stop_requested; i++) osDelay(10);
        if (stop_requested) { Motor_Brake(); continue; }
        Enc_Update(sdir);
        Enc_Reset();
        for (int i = 0; i < 150 && !stop_requested; i++) osDelay(10);
        if (stop_requested) { Motor_Brake(); continue; }
        Enc_Update(sdir);
        MotorA_Set(0);
        MotorB_Set(0);
        /* distance each wheel rolled in 1.5 s, mm, and A as % of B */
        int32_t mmA = (int32_t)((int64_t)travA * 1000 / CPC_A_x100);
        int32_t mmB = (int32_t)((int64_t)travB * 1000 / CPC_B_x100);
        int32_t pct = (mmB != 0) ? (mmA * 100 / mmB) : 0;
        int n = sprintf(tx, "SPD %ld DONE A:%ldmm B:%ldmm A/B:%ld%%\r\n", v, mmA, mmB, pct);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'G' || p[0] == 'g') && (p[1] == 'Y' || p[1] == 'y'))
      {
        for (int i = 0; i < 200 && !stop_requested; i++)
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
        if (!imu_ok) imu_ok = IMU_Init();
        Gyro_CalibrateZBias();
        heading_err = 0.0f;
        int n = Bias_Report(tx);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'Y' || p[0] == 'y') && (p[1] == 'W' || p[1] == 'w'))
      {
        float yaw = 0.0f;
        uint32_t t_old = HAL_GetTick();
        for (int i = 0; i < 1600 && !stop_requested; i++)
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
        if (d > 0 && d <= 1000) {
          Car_Turn(45.0f, TURN_LEFT, +1);
          osDelay(200);
          if (motion_status == MOTION_OK) Drive_Distance(d, +1);
          int n = sprintf(tx, "DONE %s STATUS:%s\r\n", p, Motion_Status());
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADDIST [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'D' || p[0] == 'd') && (p[1] == 'R' || p[1] == 'r'))
      {
        int32_t d = atoi(&p[2]);
        if (d > 0 && d <= 1000) {
          Car_Turn(45.0f, TURN_RIGHT, +1);
          osDelay(200);
          if (motion_status == MOTION_OK) Drive_Distance(d, +1);
          int n = sprintf(tx, "DONE %s STATUS:%s\r\n", p, Motion_Status());
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADDIST [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'X' || p[0] == 'x') && (p[1] == 'L' || p[1] == 'l'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0 && a <= 360) {
          float got = Car_Turn_Square((float)a, TURN_LEFT, -1);
          int n = sprintf(tx, "DONE %s IMU:%ld W:%ld TRIM:%ld MAX:%ld R:%ld ERR:%u SAT:%u STATUS:%s\r\n", p, (int32_t)(got * 10), (int32_t)(turn_want * 10), (int32_t)(turn_trim * 10), (int32_t)imu_max_dps, (int32_t)turn_brake_rate, imu_err_count, imu_sat_count, Motion_Status());
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR BADANGLE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      else if ((p[0] == 'X' || p[0] == 'x') && (p[1] == 'R' || p[1] == 'r'))
      {
        int32_t a = atoi(&p[2]);
        if (a > 0 && a <= 360) {
          float got = Car_Turn_Square((float)a, TURN_RIGHT, -1);
          int n = sprintf(tx, "DONE %s IMU:%ld W:%ld TRIM:%ld MAX:%ld R:%ld ERR:%u SAT:%u STATUS:%s\r\n", p, (int32_t)(got * 10), (int32_t)(turn_want * 10), (int32_t)(turn_trim * 10), (int32_t)imu_max_dps, (int32_t)turn_brake_rate, imu_err_count, imu_sat_count, Motion_Status());
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
        if (target_mm < 20 || target_mm > AC_VALID_MAX_MM) {
          int n = sprintf(tx, "ERR AC_RANGE [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100); continue;
        }
        int32_t d;
        Car_ApproachUltrasonic(target_mm, &d);
        int n = sprintf(tx, "DONE %s US:%ld MOVED:%ld STATUS:%s\r\n",
                        p, d, ap_moved_cm, Motion_Status());
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'R' || p[0] == 'r') && (p[1] == 'A' || p[1] == 'a'))
      {
        motion_status = MOTION_OK;
        int32_t back = ap_moved_cm;
        if (back > 0)       Drive_Distance(back, -1);
        else if (back < 0)  Drive_Distance(-back, +1);
        ap_moved_cm = 0;
        int n = sprintf(tx, "DONE %s UNDID:%ld STATUS:%s\r\n", p, back, Motion_Status());
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if (p[0] == 'I' && p[1] == 'E')
      {
        char side = p[2], mode = p[3];
        int32_t d = atoi(&p[4]);
        if ((side != 'L' && side != 'R') || (mode != 'N' && mode != 'A') || d <= 0 || d > 1000) {
          int n = sprintf(tx, "ERR BADARG [%s]\r\n", p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
          continue;
        }
        memset(&ie, 0, sizeof(ie));
        ie.which = (side == 'L') ? IE_LEFT_IR : IE_RIGHT_IR;
        ie.need_seen = (mode == 'N');
        ie.edge_mm = -1;
        ie.last_ir = ie.last_us = HAL_GetTick() - 1000U;   /* sample at once */
        ie.on = 1;
        Drive_Distance(d, +1);
        ie.on = 0;
        int32_t near = -1;
        if (ie.n_near > 0) {                              /* median of the near samples */
          for (int i = 1; i < ie.n_near; i++) {
            int32_t k = ie.near_mm[i]; int j = i - 1;
            while (j >= 0 && ie.near_mm[j] > k) { ie.near_mm[j + 1] = ie.near_mm[j]; j--; }
            ie.near_mm[j + 1] = k;
          }
          near = ie.near_mm[ie.n_near / 2];
        }
        static const char *ends[] = { "MAX", "EDGE", "NEVER", "WALL" };
        int n = sprintf(tx, "DONE %s END:%s EDGE:%ld NEAR:%ld NN:%d MOVED:%ld HE:%ld STATUS:%s\r\n",
                        p, ends[ie.end], ie.edge_mm, near, ie.n_near, IE_Travel_mm(),
                        (int32_t)(heading_err * 10), Motion_Status());
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if ((p[0] == 'I' || p[0] == 'i') && (p[1] == 'R' || p[1] == 'r'))
      {
        uint32_t raw1 = IR_ReadADC(1), raw2 = IR_ReadADC(2);
        int n = sprintf(tx, "IR1 raw:%lu mm:%ld  IR2 raw:%lu mm:%ld\r\n",
                        raw1, IR_RawTo_mm(raw1),
                        raw2, IR_RawTo_mm(raw2));
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if (p[0] == 'T' && p[1] == 'M') {
        int v = atoi(p + 2);
        if (v == 0 || v == 1) turn_single_wheel = v;
        int n = sprintf(tx, "%s TM:%d\r\n", (v == 0 || v == 1) ? "TURNMODE" : "ERR", turn_single_wheel);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      else if (p[0] == 'V' && p[1] == 'S') {
        int v = atoi(p + 2);
        int ok = v >= SPEED_MIN && v <= PWM_MAX;
        if (ok) drive_cruise_pwm = v;
        int n = sprintf(tx, "%s VS:%ld\r\n", ok ? "SPEED" : "ERR", drive_cruise_pwm);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
#if STEER_TEST
      /* SU<us> : hold servo at a raw pulse for 3 s, then re-centre.
                  Used to find where the steering linkage stops. */
      else if ((p[0] == 'S' || p[0] == 's') && (p[1] == 'U' || p[1] == 'u'))
      {
        int32_t us = atoi(&p[2]);
        if (us >= SERVO_US_MIN && us <= SERVO_US_MAX) {
          Servo_SetPulse((uint16_t)us);
          osDelay(3000);
          Servo_Center();
          int n = sprintf(tx, "SU %ld held 3s, recentred\r\n", us);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        } else {
          int n = sprintf(tx, "ERR SU range %d-%d [%s]\r\n", SERVO_US_MIN, SERVO_US_MAX, p);
          HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
        }
      }
      /* KL<us> / KR<us> : set left / right steering lock used by turns.
         KQ             : show current values.
         IP<pwm>        : set inner rear wheel PWM used by turns (0-4100).
         KS<pct>        : set XL speed (same as VXL).
         KH0 / KH1      : coast steering: 0 = swing to full left (old), 1 = hold the turn's lock.
         KA0 / KA1      : stop: 0 = fixed STOP_OFFSET_*, 1 = speed-adaptive (COAST_K).
         KC<us>         : straight-ahead servo pulse (1300-1700), e.g. KC1520.
         KF<us>         : pre-move steering flick (0 = none, 1000 = full left).
         KD0 / KD1      : heading hold during FW/BW off / on.
         KG<us>         : heading hold, us of steering per degree (default 20).
         KY<x10>        : heading hold, degrees of aim per cm off the line x10 (default 15).
         KT0 / KT1      : post-turn trim off / on.
         KM<pwm>        : trim creep PWM (outer wheel), e.g. KM2800.
         T<turn><ms>    : COAST_K for one turn type in milliseconds, e.g. TRT115 = 0.115 s.
         V<turn><pct>   : set speed of one turn type, 30-100 %:
                          VLT80, VRT80, VXL75, VXR75. */
      else if ((p[0] == 'K' || p[0] == 'k') || ((p[0] == 'I' || p[0] == 'i') && (p[1] == 'P' || p[1] == 'p')))
      {
        char c0 = p[0] | 0x20, c1 = p[1] | 0x20;   /* lower-case */
        int32_t v = atoi(&p[2]);
        int ok = 1;
        if (c0 == 'k' && c1 == 'l') {
          if (v >= SERVO_US_MIN && v <= SERVO_US_MAX) steer_left_us = (uint16_t)v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'r') {
          if (v >= SERVO_US_MIN && v <= SERVO_US_MAX) steer_right_us = (uint16_t)v; else ok = 0;
        } else if (c0 == 'i' && c1 == 'p') {
          if (v >= 0 && v <= TURN_PWM_OUTER) {           /* IP sets all four */
            for (int k = 0; k < 4; k++) turn_inner[k] = v;
          } else ok = 0;
        } else if (c0 == 'k' && c1 == 'a') {
          if (v == 0 || v == 1) adaptive_stop = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'd') {
          if (v == 0 || v == 1) hold_en = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'g') {
          if (v >= 0 && v <= 100) hold_k_head = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'y') {
          if (v >= 0 && v <= 100) hold_k_lat10 = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'f') {
          if (v == 0 || (v >= SERVO_US_MIN && v <= SERVO_US_MAX)) flick_us = (uint16_t)v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'c') {
          if (v >= 1300 && v <= 1700) { center_us = (uint16_t)v; Servo_Center(); } else ok = 0;
        } else if (c0 == 'k' && c1 == 't') {
          if (v == 0 || v == 1) trim_enable = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'm') {
          if (v >= 0 && v <= PWM_MAX) trim_pwm = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 'h') {
          if (v == 0 || v == 1) coast_hold = v; else ok = 0;
        } else if (c0 == 'k' && c1 == 's') {
          if (v >= 30 && v <= 100) turn_pct[2] = v; else ok = 0;
        } else if (!(c0 == 'k' && c1 == 'q')) {
          ok = 0;
        }
        /* order in each group: LT/RT/XL/XR */
        int n = sprintf(tx, "%s KL:%u KR:%u KC:%u KF:%u KD:%ld KG:%ld KY:%ld KH:%ld KA:%ld KT:%ld KM:%ld T:%ld/%ld/%ld/%ld V:%ld/%ld/%ld/%ld N:%ld/%ld/%ld/%ld C:%ld/%ld/%ld/%ld\r\n",
                        ok ? "STEER" : "ERR STEER", steer_left_us, steer_right_us, center_us, flick_us, hold_en, hold_k_head, hold_k_lat10, coast_hold, adaptive_stop, trim_enable, trim_pwm,
                        (int32_t)(coast_k[0] * 1000.0f), (int32_t)(coast_k[1] * 1000.0f),
                        (int32_t)(coast_k[2] * 1000.0f), (int32_t)(coast_k[3] * 1000.0f),
                        turn_pct[0], turn_pct[1], turn_pct[2], turn_pct[3],
                        turn_inner[0], turn_inner[1], turn_inner[2], turn_inner[3],
                        turn_comp[0], turn_comp[1], turn_comp[2], turn_comp[3]);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      /* T<turn><ms> : coast constant for one turn type, e.g. TRT115 */
      else if (p[0] == 'T' || p[0] == 't')
      {
        char a = p[1] | 0x20, b = p[2] | 0x20;
        int ti = -1;
        if      (a == 'l' && b == 't') ti = 0;
        else if (a == 'r' && b == 't') ti = 1;
        else if (a == 'x' && b == 'l') ti = 2;
        else if (a == 'x' && b == 'r') ti = 3;
        int32_t v = atoi(&p[3]);
        int ok = (ti >= 0 && v >= 0 && v <= 400);
        if (ok) coast_k[ti] = (float)v / 1000.0f;
        int n = sprintf(tx, "%s T LT:%ld RT:%ld XL:%ld XR:%ld\r\n", ok ? "COAST" : "ERR COAST",
                        (int32_t)(coast_k[0] * 1000.0f), (int32_t)(coast_k[1] * 1000.0f),
                        (int32_t)(coast_k[2] * 1000.0f), (int32_t)(coast_k[3] * 1000.0f));
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      /* N<turn><pwm> : inner rear wheel PWM for one turn type, e.g. NRT1800 */
      else if (p[0] == 'N' || p[0] == 'n')
      {
        char a = p[1] | 0x20, b = p[2] | 0x20;
        int ti = -1;
        if      (a == 'l' && b == 't') ti = 0;
        else if (a == 'r' && b == 't') ti = 1;
        else if (a == 'x' && b == 'l') ti = 2;
        else if (a == 'x' && b == 'r') ti = 3;
        int32_t v = atoi(&p[3]);
        int ok = (ti >= 0 && v >= 0 && v <= TURN_PWM_OUTER);
        if (ok) turn_inner[ti] = v;
        int n = sprintf(tx, "%s N LT:%ld RT:%ld XL:%ld XR:%ld\r\n", ok ? "INNER" : "ERR INNER",
                        turn_inner[0], turn_inner[1], turn_inner[2], turn_inner[3]);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      /* C<turn><cm> : square-turn compensation distance, e.g. CLT11, CXR0 (0 = off) */
      else if (p[0] == 'C' || p[0] == 'c')
      {
        char a = p[1] | 0x20, b = p[2] | 0x20;
        int ti = -1;
        if      (a == 'l' && b == 't') ti = 0;
        else if (a == 'r' && b == 't') ti = 1;
        else if (a == 'x' && b == 'l') ti = 2;
        else if (a == 'x' && b == 'r') ti = 3;
        int32_t v = atoi(&p[3]);
        int ok = (ti >= 0 && v >= 0 && v <= 25);
        if (ok) turn_comp[ti] = v;
        int n = sprintf(tx, "%s C LT:%ld RT:%ld XL:%ld XR:%ld\r\n", ok ? "COMP" : "ERR COMP",
                        turn_comp[0], turn_comp[1], turn_comp[2], turn_comp[3]);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
      /* V<turn><pct> : per-turn-type speed, e.g. VXR75 */
      else if (p[0] == 'V' || p[0] == 'v')
      {
        char a = p[1] | 0x20, b = p[2] | 0x20;
        int ti = -1;
        if      (a == 'l' && b == 't') ti = 0;
        else if (a == 'r' && b == 't') ti = 1;
        else if (a == 'x' && b == 'l') ti = 2;
        else if (a == 'x' && b == 'r') ti = 3;
        int32_t v = atoi(&p[3]);
        int ok = (ti >= 0 && v >= 30 && v <= 100);
        if (ok) turn_pct[ti] = v;
        int n = sprintf(tx, "%s V LT:%ld RT:%ld XL:%ld XR:%ld\r\n", ok ? "SPEED" : "ERR SPEED",
                        turn_pct[0], turn_pct[1], turn_pct[2], turn_pct[3]);
        HAL_UART_Transmit(&huart3, (uint8_t *)tx, n, 100);
      }
#endif
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

	snprintf((char*)hello, sizeof(hello), "RX:%-19.19s", aRxBuffer);
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
