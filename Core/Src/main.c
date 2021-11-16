/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>

#include <time.h>

#include <sensor_msgs/msg/joint_state.h>             // for the encoder msg
#include <geometry_msgs/msg/twist.h>                 // for the motors control
#include <std_msgs/msg/int32.h>
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* Definitions for task_ros2 */
osThreadId_t task_ros2Handle;
const osThreadAttr_t task_ros2_attributes = {
  .name = "task_ros2",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_gps */
osThreadId_t task_gpsHandle;
const osThreadAttr_t task_gps_attributes = {
  .name = "task_gps",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_stepper */
osThreadId_t task_stepperHandle;
const osThreadAttr_t task_stepper_attributes = {
  .name = "task_stepper",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_angle_sens */
osThreadId_t task_angle_sensHandle;
const osThreadAttr_t task_angle_sens_attributes = {
  .name = "task_angle_sens",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_analog_inp */
osThreadId_t task_analog_inpHandle;
const osThreadAttr_t task_analog_inp_attributes = {
  .name = "task_analog_inp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_dac */
osThreadId_t task_dacHandle;
const osThreadAttr_t task_dac_attributes = {
  .name = "task_dac",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_digital_io */
osThreadId_t task_digital_ioHandle;
const osThreadAttr_t task_digital_io_attributes = {
  .name = "task_digital_io",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_main */
osThreadId_t task_mainHandle;
const osThreadAttr_t task_main_attributes = {
  .name = "task_main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */
/* Subscriber declaration */
rcl_subscription_t cmd_vel_sub;

/* Publisher declaration */
rcl_publisher_t joint_state_pub;
rcl_publisher_t int32_pub;

/* ROS timer declaration */
rcl_timer_t timer;
rcl_timer_t timer2;

/* Messages declaration */
sensor_msgs__msg__JointState joint_state_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 int32_msg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
void task_ros2_function(void *argument);
void task_gps_function(void *argument);
void task_stepper_function(void *argument);
void task_angle_sensor_function(void *argument);
void task_analog_input_function(void *argument);
void task_dac_function(void *argument);
void task_digital_io_function(void *argument);
void task_main_function(void *argument);

/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_callback(const void * msgin);

//extern int clock_gettime( int clock_id, struct timespec * tp );
extern void UTILS_NanosecondsToTimespec( int64_t llSource, struct timespec * const pxDestination );
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

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
  /* creation of task_ros2 */
  task_ros2Handle = osThreadNew(task_ros2_function, NULL, &task_ros2_attributes);

  /* creation of task_gps */
  task_gpsHandle = osThreadNew(task_gps_function, NULL, &task_gps_attributes);

  /* creation of task_stepper */
  task_stepperHandle = osThreadNew(task_stepper_function, NULL, &task_stepper_attributes);

  /* creation of task_angle_sens */
  task_angle_sensHandle = osThreadNew(task_angle_sensor_function, NULL, &task_angle_sens_attributes);

  /* creation of task_analog_inp */
  task_analog_inpHandle = osThreadNew(task_analog_input_function, NULL, &task_analog_inp_attributes);

  /* creation of task_dac */
  task_dacHandle = osThreadNew(task_dac_function, NULL, &task_dac_attributes);

  /* creation of task_digital_io */
  task_digital_ioHandle = osThreadNew(task_digital_io_function, NULL, &task_digital_io_attributes);

  /* creation of task_main */
  task_mainHandle = osThreadNew(task_main_function, NULL, &task_main_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|STEPPER_EN_Pin|STEPPER_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin STEPPER_EN_Pin STEPPER_DIR_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|STEPPER_EN_Pin|STEPPER_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0_Pin */
  GPIO_InitStruct.Pin = PB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		// Blink the LED1 (yellow) for debugging
		//HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);

		// Fill the message timestamp
		struct timespec ts;
		int64_t time_ns;
		time_ns = rmw_uros_epoch_nanos();
		UTILS_NanosecondsToTimespec(time_ns, &ts);
		//clock_gettime(CLOCK_REALTIME, &ts);

		// Create the Header
		joint_state_msg.header.stamp.sec = ts.tv_sec;
		joint_state_msg.header.stamp.nanosec = ts.tv_nsec;

		//sprintf(joint_state_msg.header.frame_id.data, "%ld", seq_no);
		//joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);

		joint_state_msg.position.data[0] = 1;
		joint_state_msg.position.data[1] = 2;
		joint_state_msg.velocity.data[0] = 3;
		joint_state_msg.velocity.data[1] = 4;
		joint_state_msg.effort.data[0] = 5;
		joint_state_msg.effort.data[1] = 6;

		// Publish the message
		rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
		if (ret != RCL_RET_OK)
		{
		  printf("Error publishing joint_state (line %d)\n", __LINE__);
		}
	}
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{

		// Blink the LED1 (yellow) for debugging
		//HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);

		// Fill the message timestamp

		//clock_gettime(CLOCK_REALTIME, &ts);

		// Create the Header



		//sprintf(joint_state_msg.header.frame_id.data, "%ld", seq_no);
		//joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);

		int32_msg.data++;

		// Publish the message
		rcl_ret_t ret = rcl_publish(&int32_pub, &int32_msg, NULL);
		if (ret != RCL_RET_OK)
		{
		  printf("Error publishing joint_state (line %d)\n", __LINE__);
		}

}

void cmd_vel_callback(const void * msgin)
{
	double leftWheelVelocity, rightWheelVelocity;
	double linearX, linearY, linearZ, angularX, angularY, angularZ;
	const geometry_msgs__msg__Twist * cmd_vel_msg;

	if (msgin != NULL)
	{

		// Blink the LED2 (orange) for debugging
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

		cmd_vel_msg = (const geometry_msgs__msg__Twist *)msgin;

		linearX = cmd_vel_msg->linear.x;
		linearY = cmd_vel_msg->linear.y;
		linearZ = cmd_vel_msg->linear.z;
		angularX = cmd_vel_msg->angular.x;
		angularY = cmd_vel_msg->angular.y;
		angularZ = cmd_vel_msg->angular.z;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_ros2_function */
/**
  * @brief  Function implementing the task_ros2 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_ros2_function */
void task_ros2_function(void *argument)
{
  /* USER CODE BEGIN 5 */
	// micro-ROS configuration
	  rmw_uros_set_custom_transport(
		true,
		(void *) &huart2,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		  printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app
	  /*rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "acquisition_system", "", &support);
	  // It is not possible to see the nodes and topics in the ros2 terminal neither in the rqt
	  // This problem due ROS_DOMAIN_ID. To "solve" it run unset ROS_DOMAIN_ID.
	  // Details are described here https://github.com/micro-ROS/micro_ros_arduino/issues/21*/



	  /*rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;
	  rcl_init_options_t init_options;
	  size_t domain_id;

	  allocator = rcl_get_default_allocator();
	  init_options = rcl_get_zero_initialized_init_options();
	  rcl_init_options_init(&init_options, allocator);

	  domain_id = 25;
	  if(rcl_init_options_set_domain_id(&init_options, domain_id) != RCL_RET_OK)
	  {
		  printf("Error on rcl_init_options_set_domain_id (line %d)\n", __LINE__);
	  }

	  // create init_options
	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	  // create node
	  rclc_node_init_default(&node, "acquisition_system", "", &support);*/



	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;
	  rcl_init_options_t init_options;

	  allocator = rcl_get_default_allocator();
	  init_options = rcl_get_zero_initialized_init_options();
	  rcl_init_options_init(&init_options, allocator);

	  // create init_options
	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	  // create node
	  rcl_node_options_t node_ops = rcl_node_get_default_options();
	  node_ops.domain_id = 25;
	  rclc_node_init_with_options(&node, "acquisition_system", "", &support, &node_ops);


	  //time sync
	  if( rmw_uros_sync_session(1000) != RMW_RET_OK)
		  printf("Error on time sync (line %d)\n", __LINE__);

	  //int64_t time_ns;
	  //time_ns = rmw_uros_epoch_nanos();


	  // create cmd_vel_sub
	  cmd_vel_sub = rcl_get_zero_initialized_subscription();
	  rclc_subscription_init_best_effort(
			  &cmd_vel_sub,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			  "/cmd_vel");


	  // create joint_state_pub
	  rclc_publisher_init_default(
		&joint_state_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"/joint_state");


	  // int32 pub
	  rclc_publisher_init_default(
			  &int32_pub,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			  "/std_msgs_msg_Int32");

	  // joint_state message allocation. Described in https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
	  joint_state_msg.header.frame_id.capacity = 20;
	  joint_state_msg.header.frame_id.data = (char*) pvPortMalloc(joint_state_msg.header.frame_id.capacity  * sizeof(char));
	  joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);

	  joint_state_msg.name.capacity = 2;
	  joint_state_msg.name.data = (rosidl_runtime_c__String*) pvPortMalloc(joint_state_msg.name.capacity * sizeof(rosidl_runtime_c__String));
	  joint_state_msg.name.size = 2;

		joint_state_msg.name.data[0].capacity = 20;
		joint_state_msg.name.data[0].data = (char*) pvPortMalloc(joint_state_msg.name.data[0].capacity * sizeof(char));
		strcpy(joint_state_msg.name.data[0].data, "Roda_L_Joint");
		joint_state_msg.name.data[0].size = strlen(joint_state_msg.name.data[0].data);
		joint_state_msg.name.data[1].capacity = 20;
		joint_state_msg.name.data[1].data = (char*) pvPortMalloc(joint_state_msg.name.data[1].capacity* sizeof(char));
		strcpy(joint_state_msg.name.data[1].data, "Roda_R_Joint");
		joint_state_msg.name.data[1].size = strlen(joint_state_msg.name.data[1].data);
		joint_state_msg.name.size=2;

		joint_state_msg.position.capacity = 2;
		joint_state_msg.position.data = (double*) pvPortMalloc(joint_state_msg.position.capacity * sizeof(double));
		joint_state_msg.position.data[0] = 0;
		joint_state_msg.position.data[1] = 0;
		joint_state_msg.position.size = 2;

	  joint_state_msg.velocity.capacity = 2;
	  joint_state_msg.velocity.data = (double*) pvPortMalloc(joint_state_msg.velocity.capacity * sizeof(double));
	  joint_state_msg.velocity.data[0] = 0;
	  joint_state_msg.velocity.data[1] = 0;
	  joint_state_msg.velocity.size = 2;

	  joint_state_msg.effort.capacity = 2;
	  joint_state_msg.effort.data = (double*) pvPortMalloc(joint_state_msg.effort.capacity * sizeof(double));
	  joint_state_msg.effort.data[0]=-1;
	  joint_state_msg.effort.data[1]=-1;
	  joint_state_msg.effort.size = 2;




	  // Create a timer
	  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback);
	  rclc_timer_init_default(&timer2, &support, RCL_MS_TO_NS(1000), timer2_callback);

	  // Create executor
	  rclc_executor_init(&executor, &support.context, 3, &allocator);
	  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
			  &cmd_vel_callback, ON_NEW_DATA); // ON_NEW_DATA does not work properly
	  rclc_executor_add_timer(&executor, &timer);
	  rclc_executor_add_timer(&executor, &timer2);
	  int32_msg.data = 0;

	  // Run executor
	  rclc_executor_spin(&executor);

	  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(1);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_gps_function */
/**
* @brief Function implementing the task_gps thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_gps_function */
void task_gps_function(void *argument)
{
  /* USER CODE BEGIN task_gps_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_gps_function */
}

/* USER CODE BEGIN Header_task_stepper_function */
/**
* @brief Function implementing the task_stepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_stepper_function */
void task_stepper_function(void *argument)
{
  /* USER CODE BEGIN task_stepper_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_stepper_function */
}

/* USER CODE BEGIN Header_task_angle_sensor_function */
/**
* @brief Function implementing the task_angle_sens thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_angle_sensor_function */
void task_angle_sensor_function(void *argument)
{
  /* USER CODE BEGIN task_angle_sensor_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_angle_sensor_function */
}

/* USER CODE BEGIN Header_task_analog_input_function */
/**
* @brief Function implementing the task_analog_inp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_analog_input_function */
void task_analog_input_function(void *argument)
{
  /* USER CODE BEGIN task_analog_input_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_analog_input_function */
}

/* USER CODE BEGIN Header_task_dac_function */
/**
* @brief Function implementing the task_dac thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_dac_function */
void task_dac_function(void *argument)
{
  /* USER CODE BEGIN task_dac_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_dac_function */
}

/* USER CODE BEGIN Header_task_digital_io_function */
/**
* @brief Function implementing the task_digital_io thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_digital_io_function */
void task_digital_io_function(void *argument)
{
  /* USER CODE BEGIN task_digital_io_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_digital_io_function */
}

/* USER CODE BEGIN Header_task_main_function */
/**
* @brief Function implementing the task_main thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_main_function */
void task_main_function(void *argument)
{
  /* USER CODE BEGIN task_main_function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task_main_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
