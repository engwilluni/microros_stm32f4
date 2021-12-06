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
#include <math.h>
#include "stepper.h"


#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/float32.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float time;
	float latitude;
	float longitude;
	float hdop;
	float altitude;
	float geoid;
	char ns;
	char ew;
	char unit;
	int lock;
	int sats;
} gps_t;

typedef struct {
	uint32_t digital_input;
	uint32_t analog_input[2];
} sensor_data_t;


typedef struct {
	uint32_t stepper_target_position;
	uint32_t digital_output;
}ros_cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TF_GPS_DATA 1<<0
#define TF_STEPPER_INT 1<<0
#define TF_STEPPER_DATA 1<<1
#define TF_MAIN_ROS_CMD 1<<0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_stepper */
osThreadId_t task_stepperHandle;
const osThreadAttr_t task_stepper_attributes = {
  .name = "task_stepper",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for task_main */
osThreadId_t task_mainHandle;
const osThreadAttr_t task_main_attributes = {
  .name = "task_main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for stepperMutex */
osMutexId_t stepperMutexHandle;
const osMutexAttr_t stepperMutex_attributes = {
  .name = "stepperMutex"
};
/* Definitions for sensorDataMutex */
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};
/* USER CODE BEGIN PV */
/* Subscriber declaration */

rcl_subscription_t ros2_gpio_output_sub;
rcl_subscription_t ros2_stepper_target_position_sub;

/* Publisher declaration */

rcl_publisher_t ros2_gpio_input_pub;
rcl_publisher_t ros2_gps_pub;
rcl_publisher_t ros2_analog_input_pub;
rcl_publisher_t ros2_stepper_current_position_pub;
rcl_publisher_t ros2_stepper_speed_pub;
/* ROS timer declaration */

rcl_timer_t ros2_gpio_input_timer;
rcl_timer_t ros2_gps_timer;
rcl_timer_t ros2_analog_input_timer;
rcl_timer_t ros2_stepper_current_position_timer;
rcl_timer_t ros2_stepper_speed_timer;

/* Messages declaration */


std_msgs__msg__UInt32 ros2_gpio_input_msg;
sensor_msgs__msg__NavSatFix ros2_gps_msg;
std_msgs__msg__UInt16MultiArray ros2_analog_input_msg;
std_msgs__msg__UInt32 ros2_gpio_output_msg;
std_msgs__msg__UInt32 ros2_stepper_target_position_msg;
std_msgs__msg__UInt32 ros2_stepper_current_position_msg;
std_msgs__msg__Float32 ros2_stepper_speed_msg;

uint16_t adc_values[2];
uint32_t gpio_input;
uint8_t gps_buffer[256];
uint8_t gps_buffer_index = 0;
uint8_t uart_gps_rx = 0;



stepper_t stepper;
gps_t gps;
ros_cmd_t ros_cmd;
sensor_data_t sensor_data;


HAL_StatusTypeDef status;



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
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void task_ros2_function(void *argument);
void task_gps_function(void *argument);
void task_stepper_function(void *argument);
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


void ros2_gpio_input_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void ros2_gps_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void ros2_analog_input_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void ros2_stepper_current_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void ros2_stepper_speed_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

void ros2_gpio_output_callback(const void * msgin);
void ros2_stepper_target_position_callback(const void * msgin);

//extern int clock_gettime( int clock_id, struct timespec * tp );
extern void UTILS_NanosecondsToTimespec( int64_t llSource, struct timespec * const pxDestination );
double convertDegMinToDecDeg (float degMin);
void updateDigitalOuputs(uint32_t digital_output);
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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, adc_values, 2);
  status = HAL_UART_Receive_IT(&huart3, &uart_gps_rx, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of stepperMutex */
  stepperMutexHandle = osMutexNew(&stepperMutex_attributes);

  /* creation of sensorDataMutex */
  sensorDataMutexHandle = osMutexNew(&sensorDataMutex_attributes);

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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|STEPPER_EN_Pin|STEPPER_DIR_Pin|MS1_Pin
                          |MS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin STEPPER_EN_Pin STEPPER_DIR_Pin MS1_Pin
                           MS2_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|STEPPER_EN_Pin|STEPPER_DIR_Pin|MS1_Pin
                          |MS2_Pin;
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

/**
 * Convert NMEA absolute position to decimal degrees
 * "ddmm.mmmm" or "dddmm.mmmm" really is D+M/60,
 * then negated if quadrant is 'W' or 'S'
 */
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}

void ros2_gpio_input_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

		ros2_gpio_input_msg.data= sensor_data.digital_input;

		// Publish the message
		rcl_ret_t ret = rcl_publish(&ros2_gpio_input_pub, &ros2_gpio_input_msg, NULL);
		if (ret != RCL_RET_OK)
		{
		  printf("Error publishing gpio inputs (line %d)\n", __LINE__);
		}
}

void ros2_gps_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	if (timer != NULL) {
// Fill the message timestamp
		struct timespec ts;
		int64_t time_ns;
		time_ns = rmw_uros_epoch_nanos();
		UTILS_NanosecondsToTimespec(time_ns, &ts);


	// Create the Header
		ros2_gps_msg.header.stamp.sec = ts.tv_sec;
		ros2_gps_msg.header.stamp.nanosec = ts.tv_nsec;

		ros2_gps_msg.latitude = gps.latitude;
		ros2_gps_msg.longitude = gps.longitude;
		ros2_gps_msg.altitude = gps.altitude;
		// Publish the message
				rcl_ret_t ret = rcl_publish(&ros2_gps_pub, &ros2_gps_msg, NULL);
				if (ret != RCL_RET_OK)
				{
				  printf("Error publishing gps (line %d)\n", __LINE__);
				}
			}


}

void ros2_analog_input_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	if (timer != NULL) {
		osMutexAcquire(sensorDataMutexHandle, osWaitForever);
	ros2_analog_input_msg.data.data[0] = sensor_data.analog_input[0];
	ros2_analog_input_msg.data.data[1] = sensor_data.analog_input[1];
	osMutexRelease(sensorDataMutexHandle);
	// Publish the message
					rcl_ret_t ret = rcl_publish(&ros2_analog_input_pub, &ros2_analog_input_msg, NULL);
					if (ret != RCL_RET_OK)
					{
					  printf("Error publishing analog input (line %d)\n", __LINE__);
					}
	}

}

void ros2_stepper_speed_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	if (timer != NULL) {

	ros2_stepper_speed_msg.data = stepper.speed;

	// Publish the message
					rcl_ret_t ret = rcl_publish(&ros2_stepper_speed_pub, &ros2_stepper_speed_msg, NULL);
					if (ret != RCL_RET_OK)
					{
					  printf("Error publishing stepper speed (line %d)\n", __LINE__);
					}
	}
}

void ros2_stepper_current_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
	if (timer != NULL) {

	ros2_stepper_current_position_msg.data = stepper.currentPos;

	// Publish the message
					rcl_ret_t ret = rcl_publish(&ros2_stepper_current_position_pub, &ros2_stepper_current_position_msg, NULL);
					if (ret != RCL_RET_OK)
					{
					  printf("Error publishing stepper current position (line %d)\n", __LINE__);
					}
	}
}




void ros2_stepper_target_position_callback(const void * msgin){
	const std_msgs__msg__UInt32 *stepper_target_position_msg;

		if (msgin != NULL)
		{


			stepper_target_position_msg = (const std_msgs__msg__UInt32 *)msgin;

			ros_cmd.stepper_target_position = stepper_target_position_msg->data;

			osThreadFlagsSet(task_mainHandle, TF_MAIN_ROS_CMD);

		}
}

void ros2_gpio_output_callback(const void * msgin)
{

	const std_msgs__msg__UInt32 *gpio_output_msg;
	int32_t data = 0;
	if (msgin != NULL)
	{



		gpio_output_msg = (const std_msgs__msg__UInt32 *)msgin;
		data = gpio_output_msg->data;

		ros_cmd.digital_output = data;
		osThreadFlagsSet(task_mainHandle, TF_MAIN_ROS_CMD);




	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
if (huart->Instance == USART3){
	if ((uart_gps_rx != '\n') && gps_buffer_index < sizeof(gps_buffer)){
		gps_buffer[gps_buffer_index++] = uart_gps_rx;
	} else {
		osThreadFlagsSet(task_gpsHandle, TF_GPS_DATA);
	}

	status = HAL_UART_Receive_IT(&huart3, &uart_gps_rx, 1);

}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		if (stepper.direction == DIRECTION_CW) {
			stepper.currentPos++;
			HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_SET);
		} else {
			stepper.currentPos--;
			HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);
		}
		osThreadFlagsSet(task_stepperHandle, TF_STEPPER_INT);

	}
}

void updateDigitalOuputs(uint32_t digital_output){
	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, (digital_output & 0x0001)>> 0);
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, (digital_output & 0x0002)>> 1);
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


	  //create gpio_output_sub
	  ros2_gpio_output_sub = rcl_get_zero_initialized_subscription();
	  rclc_subscription_init_best_effort(
			  &ros2_gpio_output_sub,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
			  "/gpio_output");

	  //create stepper_target_positoin_sub
	  ros2_stepper_target_position_sub = rcl_get_zero_initialized_subscription();
	 	  rclc_subscription_init_best_effort(
	 			  &ros2_stepper_target_position_sub,
	 			  &node,
	 			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
	 			  "/stepper_target_position");



	  // gpio_input pub
	  rclc_publisher_init_default(
			  &ros2_gpio_input_pub,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
			  "/gpio_input");

	  // gps_pub
	  rclc_publisher_init_default(
	  			  &ros2_gps_pub,
	  			  &node,
	  			  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
	  			  "/gps");

	  rclc_publisher_init_default(
	  			  &ros2_analog_input_pub,
	  			  &node,
	  			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
	  			  "/analog_input");


	  rclc_publisher_init_default(
	  	  			  &ros2_stepper_current_position_pub,
	  	  			  &node,
	  	  			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
	  	  			  "/stepper_current_position");

	  rclc_publisher_init_default(
		  	  			  &ros2_stepper_speed_pub,
		  	  			  &node,
		  	  			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		  	  			  "/stepper_speed");

	  // gps memmory allocation
	  ros2_gps_msg.header.frame_id.capacity = 20;
	  ros2_gps_msg.header.frame_id.data = (char*) pvPortMalloc(ros2_gps_msg.header.frame_id.capacity  * sizeof(char));
	  ros2_gps_msg.header.frame_id.size = strlen(ros2_gps_msg.header.frame_id.data);


	  // analog input allocation
	  ros2_analog_input_msg.data.capacity = 2;
	  ros2_analog_input_msg.data.size = 2;
	  ros2_analog_input_msg.data.data = (uint16_t*) pvPortMalloc(ros2_analog_input_msg.data.capacity * sizeof(uint16_t));
	  ros2_analog_input_msg.layout.dim.capacity = 2;
	  ros2_analog_input_msg.layout.dim.size = 2;
	  ros2_analog_input_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) pvPortMalloc(ros2_analog_input_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
	  for (size_t i =0; i< ros2_analog_input_msg.layout.dim.capacity; i++){
		  ros2_analog_input_msg.layout.dim.data[i].label.capacity = 20;
		  ros2_analog_input_msg.layout.dim.data[i].label.size = 10;
		  ros2_analog_input_msg.layout.dim.data[i].label.data = (char*) pvPortMalloc(ros2_analog_input_msg.layout.dim.data[i].label.capacity * sizeof(char));

	  }
	  strcpy(ros2_analog_input_msg.layout.dim.data[0].label.data, "Analog 1");
	  strcpy(ros2_analog_input_msg.layout.dim.data[1].label.data, "Analog 2");


	  // Create a timer
	  rclc_timer_init_default(&ros2_gpio_input_timer, &support, RCL_MS_TO_NS(100), ros2_gpio_input_timer_callback);
	  rclc_timer_init_default(&ros2_gps_timer, &support, RCL_MS_TO_NS(1000), ros2_gps_timer_callback);
	  rclc_timer_init_default(&ros2_analog_input_timer, &support, RCL_MS_TO_NS(100), ros2_analog_input_timer_callback);
	  rclc_timer_init_default(&ros2_stepper_current_position_timer, &support, RCL_MS_TO_NS(100), ros2_stepper_current_position_timer_callback);
	  rclc_timer_init_default(&ros2_stepper_speed_timer, &support, RCL_MS_TO_NS(100), ros2_stepper_speed_timer_callback);


	  // Create executor
	  rclc_executor_init(&executor, &support.context, 7, &allocator);

	  rclc_executor_add_subscription(&executor, &ros2_gpio_output_sub, &ros2_gpio_output_msg,
	  			  &ros2_gpio_output_callback, ON_NEW_DATA); // ON_NEW_DATA does not work properly
	  rclc_executor_add_subscription(&executor, &ros2_stepper_target_position_sub, &ros2_stepper_target_position_msg,
	 	  			  &ros2_stepper_target_position_callback, ON_NEW_DATA); // ON_NEW_DATA does not work properly

	  rclc_executor_add_timer(&executor, &ros2_gpio_input_timer);
	  rclc_executor_add_timer(&executor, &ros2_gps_timer);
	  rclc_executor_add_timer(&executor, &ros2_analog_input_timer);
	  rclc_executor_add_timer(&executor, &ros2_stepper_speed_timer);
	  rclc_executor_add_timer(&executor, &ros2_stepper_current_position_timer);



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
    osThreadFlagsWait(TF_GPS_DATA, osFlagsWaitAny, osWaitForever);
    if(sscanf(gps_buffer, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f", &gps.time, &gps.latitude, &gps.ns, &gps.longitude, &gps.ew, &gps.lock, &gps.sats, &gps.hdop, &gps.altitude, &gps.unit, &gps.geoid) >= 1){
    	gps.latitude = (float) convertDegMinToDecDeg(gps.latitude);
    	gps.longitude = (float) convertDegMinToDecDeg(gps.longitude);
    	if (gps.ns == 'S') {
    		gps.latitude = - gps.latitude;
    	}
    	if (gps.ew = 'W'){
    		gps.longitude = - gps.longitude;
    	}
    }
    gps_buffer_index = 0;
    memset(gps_buffer, 0, sizeof(gps_buffer));
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
	uint32_t flags;
	stepperInit(&stepper);

	stepperSetSpeed(&stepper, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, stepper.stepInverval);
	stepperSetAcceleration(&stepper, 48*2*4);
	// Motor = 7.5°/Step, 1/4 Microstepping, speed =  2rev/s,
	stepperSetMaxSpeed(&stepper, 2*4*48);
	stepperSetAbsoluteTartePosition(&stepper, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim3, stepper.stepInverval);
	 HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {

	  flags = osThreadFlagsWait((TF_STEPPER_INT | TF_STEPPER_DATA), osFlagsWaitAny, osWaitForever);
	  if ((flags & TF_STEPPER_INT) == TF_STEPPER_INT){
		  stepperComputeNewSpeed(&stepper);
		  __HAL_TIM_SET_AUTORELOAD(&htim3, stepper.stepInverval);
	  }

	  if ((flags & TF_STEPPER_DATA) == TF_STEPPER_DATA){
		  stepperComputeNewSpeed(&stepper);
		  __HAL_TIM_SET_AUTORELOAD(&htim3, stepper.stepInverval);
	  }
  }
  /* USER CODE END task_stepper_function */
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
	uint32_t flags  = 0;
  /* Infinite loop */
  for(;;)
  {

	  flags = osThreadFlagsWait(TF_MAIN_ROS_CMD, osFlagsWaitAny, 0);
	  if ((flags & TF_MAIN_ROS_CMD) == TF_MAIN_ROS_CMD){
		  stepper.targetPos = ros_cmd.stepper_target_position;
		  osThreadFlagsSet(task_stepperHandle, TF_STEPPER_DATA);
		  updateDigitalOuputs(ros_cmd.digital_output);

	  }
	  osMutexAcquire(sensorDataMutexHandle, osWaitForever);
	  sensor_data.digital_input = HAL_GPIO_ReadPin(PB0_GPIO_Port, PB0_Pin);
	  sensor_data.analog_input[0] = adc_values[0];
	  sensor_data.analog_input[1] = adc_values[1];
	  osMutexRelease(sensorDataMutexHandle);

    osDelay(50);
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
