/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_hal_i2c.h"

#include "arm_math.h"
#include "attitude_ekf.h"
#include "attitude_measurement.h"

#include "bmm350.h"
#include "bmi08x.h"
#include "bno055.h"

#include "dynamixel/dynamixel.h"
#include "dynamixel_ll_uart.h"

#include "hexapodmath/additional_functions.h"


#include "robot.h"
#include "calculator.h"
#include "servos.h"
#include "log.h"
#include "semphr.h"
#include "dynamixel/protocol.h"
#include "controller.h"
#include "event_groups.h"
#include "rgb_led.h"
#include "measure.h"
#include "sensors.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct bmm350_dev bmm350_t;
typedef struct bmi08_dev bmi08_t;
typedef struct bno055_t bno055_tt;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
} i2c_intf_ptr;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CS_Port;
    uint16_t CS_Pin;
} spi_intf_ptr;

typedef struct {
    float roll, pitch, yaw;
    float gyro_bias[3];
    TickType_t last_update_ts;
} ekf_output_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DYNAMIXEL_ERROR_CHECK(x) do { \
        dynamixel_result_t dynamixel_rc_ = (x); \
        if (dynamixel_rc_ != DNM_OK) { \
            LOG_ERROR("Dynamixel call returned failure %d", dynamixel_rc_); \
            Error_Handler(); \
        } \
    } while(0)

#define MATRIX(M,S) \
arm_matrix_instance_f32 M; \
float32_t p ## M ## Data[S*S];   \
arm_mat_init_f32(&M, S, S, p ## M ## Data)

#define MATRIX4(M) \
MATRIX(M, 4)

#define MAIN_LOOP_INTERVAL 200.0f // ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiSlaveTask */
osThreadId_t spiSlaveTaskHandle;
const osThreadAttr_t spiSlaveTask_attributes = {
  .name = "spiSlaveTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for magTask */
osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
  .name = "magTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ekfTask */
osThreadId_t ekfTaskHandle;
const osThreadAttr_t ekfTask_attributes = {
  .name = "ekfTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ekf_queue */
osMessageQueueId_t ekf_queueHandle;
const osMessageQueueAttr_t ekf_queue_attributes = {
  .name = "ekf_queue"
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
const osMutexAttr_t spiMutex_attributes = {
  .name = "spiMutex"
};
/* Definitions for systemEvents */
osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};
/* USER CODE BEGIN PV */
i2c_intf_ptr bmm350_intf;
bmm350_t bmm350;

spi_intf_ptr bmi088_acc_intf;
spi_intf_ptr bmi088_gyr_intf;
bmi08_t bmi088;

i2c_intf_ptr bno055_intf;
bno055_tt bno055;

dynamixel_ll_uart_context dynamixel_uart_context;
dynamixel_bus_t dynamixel_bus;
dynamixel_servo_t dynamixel_servo[3 * 6];

volatile osThreadId_t servoCallbackThreadId;

float32_t velocity = 0.0f; // mm/s
float32_t heading = 0.0f; // rad
float32_t height = 100.f; // mm, body height from ground

// We only update at the start of the loop, store the new values here
float32_t updated_velocity = 0.f;
float32_t updated_heading = 0.f;
float32_t updated_height = 100.f;

ekf_output_t ekf_out;

uint8_t gyro_interrupt_enable = 0;
uint8_t accel_interrupt_enable = 0;

float32_t gyro_snapshot[3];
float32_t accel_snapshot[3];
float32_t mag_snapshot[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartSpiSlaveTask(void *argument);
void StartGyroTask(void *argument);
void StartAccelTask(void *argument);
void StartMagTask(void *argument);
void StartEkfTask(void *argument);

/* USER CODE BEGIN PFP */
int __io_putchar(int ch);

BMM350_INTF_RET_TYPE stm32_bmm350_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

BMM350_INTF_RET_TYPE stm32_bmm350_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

void stm32_bmm350_delay_us(uint32_t period, void *intf_ptr);

BMI08_INTF_RET_TYPE stm32_bmi08_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

BMI08_INTF_RET_TYPE stm32_bmi08_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

void stm32_bmi08_delay_us(uint32_t period, void *intf_ptr);

int8_t stm32_bno055_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t wr_len);

int8_t stm32_bno055_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t r_len);

void stm32_bno055_delay_us(u32 period);

void PERIF_BMI088_Init();
void PERIF_BMM350_Init();
void PERIF_BNO055_Init();
void PERIF_Dynamixel_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int g_log_level = LOG_LEVEL_DEBUG;

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim1);

    LOG_INFO("[Main] Hexapod Control Firmware");
    LOG_INFO("[Main] Build: %s %s", __DATE__, __TIME__);

    LOG_INFO("[Main] Starting peripheral init");
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_RESET);

    PERIF_BMI088_Init();
    PERIF_BMM350_Init();
    // PERIF_BNO055_Init();

    LOG_INFO("[Main] Peripheral init complete");
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of spiMutex */
  spiMutexHandle = osMutexNew(&spiMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ekf_queue */
  ekf_queueHandle = osMessageQueueNew (32, sizeof(sensor_sample_t), &ekf_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of spiSlaveTask */
  spiSlaveTaskHandle = osThreadNew(StartSpiSlaveTask, NULL, &spiSlaveTask_attributes);

  /* creation of gyroTask */
  gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);

  /* creation of accelTask */
  accelTaskHandle = osThreadNew(StartAccelTask, NULL, &accelTask_attributes);

  /* creation of magTask */
  magTaskHandle = osThreadNew(StartMagTask, NULL, &magTask_attributes);

  /* creation of ekfTask */
  ekfTaskHandle = osThreadNew(StartEkfTask, NULL, &ekfTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    rgb_led_init();
  /* USER CODE END RTOS_THREADS */

  /* creation of systemEvents */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF - 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI2_CS_ACC_Pin|SPI2_CS_GYR_Pin|ST_LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST_LED_B_Pin|ST_LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI2_CS_ACC_Pin SPI2_CS_GYR_Pin ST_LED_G_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_ACC_Pin|SPI2_CS_GYR_Pin|ST_LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_INT_ACC_Pin SPI2_INT_GYR_Pin */
  GPIO_InitStruct.Pin = SPI2_INT_ACC_Pin|SPI2_INT_GYR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ST_LED_B_Pin ST_LED_R_Pin */
  GPIO_InitStruct.Pin = ST_LED_B_Pin|ST_LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define DEBUG_USART USART1
int __io_putchar(int ch) {
    while (!LL_USART_IsActiveFlag_TXE(DEBUG_USART)) {
    }

    LL_USART_TransmitData8(DEBUG_USART, ch);

    return ch;
}

BMM350_INTF_RET_TYPE stm32_bmm350_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL) {
        return -1;
    }

    i2c_intf_ptr *i2c_intf = (i2c_intf_ptr *) intf_ptr;
    uint16_t address = (uint16_t) i2c_intf->address << 1;

    if (HAL_I2C_Master_Transmit(i2c_intf->hi2c, address, &reg_addr, 1, 25) != HAL_OK) {
        return -2;
    }

    if (HAL_I2C_Master_Receive(i2c_intf->hi2c, address, reg_data, len, 25) != HAL_OK) {
        return -3;
    }

    return 0;
}

BMM350_INTF_RET_TYPE stm32_bmm350_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL) {
        return -1;
    }

    i2c_intf_ptr *i2c_intf = (i2c_intf_ptr *) intf_ptr;
    uint16_t address = (uint16_t) i2c_intf->address << 1;

    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);

    if (HAL_I2C_Master_Transmit(i2c_intf->hi2c, address, buffer, len + 1, 25) != HAL_OK) {
        return -2;
    }

    return 0;
}

void stm32_bmm350_delay_us(uint32_t period, void *intf_ptr) {
    (void) intf_ptr;
    // htim1 setup, prescaler 16-1, ARR 0xffff-1
    __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < period); // wait for the counter to reach the us input in the parameter
}

BMI08_INTF_RET_TYPE stm32_bmi08_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL) {
        return -1;
    }

    spi_intf_ptr *spi_intf = (spi_intf_ptr *) intf_ptr;

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(spi_intf->hspi, &reg_addr, 1, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY);

    HAL_SPI_Receive(spi_intf->hspi, reg_data, len, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY);

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_SET);

    return 0;
}

BMI08_INTF_RET_TYPE stm32_bmi08_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL) {
        return -1;
    }

    spi_intf_ptr *spi_intf = (spi_intf_ptr *) intf_ptr;

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(spi_intf->hspi, &reg_addr, 1, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY);

    HAL_SPI_Transmit(spi_intf->hspi, (uint8_t *) reg_data, len, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY);

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_SET);

    return 0;
}

void stm32_bmi08_delay_us(uint32_t period, void *intf_ptr) {
    (void) intf_ptr;
    // htim1 setup, prescaler 16-1, ARR 0xffff-1
    __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < period); // wait for the counter to reach the us input in the parameter
}

/**
 * Scales the raw values from the BMI088 accelerometer to mg
 *
 * @param raw The raw sensor value
 * @param accel_range The range configured in the sensor
 * @return The scaled value in mg
 */
static inline float32_t stm32_bmi08a_scale_data(int16_t raw, uint8_t accel_range) {
    return (float32_t)raw * 1500.0f * (float32_t)(1 << (accel_range + 1)) / 32768.0f;
}

static inline float32_t stm32_bmi08g_scale_data(int16_t raw, uint8_t gyro_range)
{
    const float base_lsb_per_dps = 16.384f;
    float32_t lsb_per_dps = base_lsb_per_dps * (float32_t)(1 << gyro_range);
    return (float)raw / lsb_per_dps;
}

/**
 * Convert accel sensor frame to NED Frame
 * On the PCB the sensor is rotated 90deg counter clock wise.
 *
 * @param sensor The measured values in sensor frame
 * @param ned The measure values in the NED frame
 */
static inline void stm32_bmi08a_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]) {
    ned[0] = sensor[1];
    ned[1] = -sensor[0];
    ned[2] = sensor[2];
}

/**
 * Convert gyro sensor frame to NED Frame
 * On the PCB the sensor is rotated 90deg counter clock wise.
 *
 * @param sensor The measured values in sensor frame
 * @param ned The measure values in the NED frame
 */
static inline void stm32_bmi08g_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]) {
    ned[0] = -sensor[1];
    ned[1] = sensor[0];
    ned[2] = -sensor[2];
}

static inline void stm32_bmm350_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]) {
    ned[0] = -sensor[0];
    ned[1] = -sensor[1];
    ned[2] = sensor[2];
}

int8_t stm32_bno055_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t wr_len) {
    uint16_t address = (uint16_t) dev_addr << 1;

    uint8_t buffer[wr_len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, wr_len);

    if (HAL_I2C_Master_Transmit(&hi2c3, address, buffer, wr_len + 1, 25) != HAL_OK) {
        return -2;
    }

    return 0;
}

int8_t stm32_bno055_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t r_len) {
    uint16_t address = (uint16_t) dev_addr << 1;

    if (HAL_I2C_Master_Transmit(&hi2c3, address, &reg_addr, 1, 25) != HAL_OK) {
        return -2;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, address, reg_data, r_len, 25) != HAL_OK) {
        return -3;
    }

    return 0;
}

void stm32_bno055_delay_us(u32 period) {
    // htim1 setup, prescaler 16-1, ARR 0xffff-1
    __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < period); // wait for the counter to reach the us input in the parameter
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        return;
    }

    if (servoCallbackThreadId == NULL) {
        LOG_DEBUG("HAL_UART_ErrorCallback: huart6 TxCplt callback, but no servoCallbackThreadId set");
        return;
    }

    osThreadFlagsSet(servoCallbackThreadId, DYNAMIXEL_DMA_TX_CPLT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        return;
    }

    if (servoCallbackThreadId == NULL) {
        LOG_DEBUG("HAL_UART_ErrorCallback: huart6 RxCplt callback, but no servoCallbackThreadId set");
        return;
    }

    osThreadFlagsSet(servoCallbackThreadId, DYNAMIXEL_DMA_RX_CPLT);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        return;
    }

    if (servoCallbackThreadId == NULL) {
        LOG_DEBUG("HAL_UART_ErrorCallback: huart6 error, but no servoCallbackThreadId set");
        return;
    }

    osThreadFlagsSet(servoCallbackThreadId, DYNAMIXEL_DMA_ERR);
}

// Callback called by HAL when SPI receive complete (in ISR context)
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    osThreadFlagsSet(spiSlaveTaskHandle, SPI1_RX_CPLT);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    osThreadFlagsSet(spiSlaveTaskHandle, SPI1_ERROR);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    UNUSED(GPIO_Pin);

    switch (GPIO_Pin) {
        case SPI2_INT_GYR_Pin: {
            if (!gyro_interrupt_enable) {
                break;
            }
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(gyroTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        }
        case SPI2_INT_ACC_Pin: {
            if (!accel_interrupt_enable) {
                break;
            }
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(accelTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
        }
        default:
            break;
    }
}

static void stm32_state_change_cb(
    controller_ctx_t *ctx,
    controller_state_t from,
    controller_state_t to,
    void *user_data
) {
    (void)ctx;
    (void)user_data;
    (void)from;

    switch (to) {
        case CTRL_SYNCING:
            rgb_led_set_color(RGB_LED_COLOR_RED);
            rgb_led_blink(500, 0.5f);
            LOG_INFO("Activating torque on servos");
            for (int i = 0; i < 3 * 6; i++) {
                dynamixel_set_torque_enable(&dynamixel_servo[i], 1);
                dynamixel_set_led(&dynamixel_servo[i], 1);
            }
            break;
        case CTRL_POWERDOWN:
            rgb_led_set_color(RGB_LED_COLOR_MAGENTA);
            rgb_led_blink(500, 0.5f);
            LOG_INFO("Deactivating torque on servos");
            for (int i = 0; i < 3 * 6; i++) {
                dynamixel_set_torque_enable(&dynamixel_servo[i], 0);
                dynamixel_set_led(&dynamixel_servo[i], 0);
            }
            break;
        case CTRL_STANDUP:
            rgb_led_set_color(RGB_LED_COLOR_CYAN);
            rgb_led_blink(500, 0.5f);
            break;
        case CTRL_STANDING:
            rgb_led_set_color(RGB_LED_COLOR_GREEN);
            rgb_led_blink(500, 0.5f);
            break;
        case CTRL_WALKING:
            rgb_led_set_color(RGB_LED_COLOR_BLUE);
            rgb_led_blink(500, 0.5f);
            break;
        default:
            break;
    }
}

/**
 * Initialize and configure the BMI088 chip
 * Deactivate interrupts for now
 *
 * On error jump to Error_Handler
 */
void PERIF_BMI088_Init() {
    // Set the two chip select lines high
    HAL_GPIO_WritePin(SPI2_CS_ACC_GPIO_Port, SPI2_CS_ACC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS_GYR_GPIO_Port, SPI2_CS_GYR_Pin, GPIO_PIN_SET);

    /* Setup BMI088 */
    bmi088_acc_intf.CS_Port = SPI2_CS_ACC_GPIO_Port;
    bmi088_acc_intf.CS_Pin = SPI2_CS_ACC_Pin;
    bmi088_acc_intf.hspi = &hspi2;

    bmi088_gyr_intf.CS_Port = SPI2_CS_GYR_GPIO_Port;
    bmi088_gyr_intf.CS_Pin = SPI2_CS_GYR_Pin;
    bmi088_gyr_intf.hspi = &hspi2;

    bmi088.variant = BMI088_VARIANT;
    bmi088.intf = BMI08_SPI_INTF;
    bmi088.delay_us = &stm32_bmi08_delay_us;
    bmi088.read = &stm32_bmi08_read;
    bmi088.write = &stm32_bmi08_write;
    bmi088.intf_ptr_accel = &bmi088_acc_intf;
    bmi088.intf_ptr_gyro = &bmi088_gyr_intf;

    // Gyro config 1000hz
    bmi088.gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS;
    bmi088.gyro_cfg.odr = BMI08_GYRO_BW_12_ODR_100_HZ;
    bmi088.gyro_cfg.bw = BMI08_GYRO_BW_12_ODR_100_HZ;
    bmi088.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    struct bmi08_gyro_int_channel_cfg gyro_int_cfg = {
        .int_channel = BMI08_INT_CHANNEL_3,
        .int_type = BMI08_GYRO_INT_DATA_RDY,
        .int_pin_cfg = {
            BMI08_INT_ACTIVE_LOW,
            BMI08_INT_MODE_PUSH_PULL,
            BMI08_DISABLE
        }
    };

    // Accel config 200hz
    bmi088.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    bmi088.accel_cfg.odr = BMI08_ACCEL_ODR_12_5_HZ;
    bmi088.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    bmi088.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    struct bmi08_accel_int_channel_cfg accel_int_cfg = {
        BMI08_INT_CHANNEL_1,
        BMI08_ACCEL_INT_DATA_RDY,
        {
            BMI08_INT_ACTIVE_LOW,
            BMI08_INT_MODE_PUSH_PULL,
            BMI08_DISABLE,
        }
    };

    int8_t bmi088_res = bmi08g_init(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 gyro initialization failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 gyro initialization complete!");
    }

    bmi088_res = bmi08g_set_meas_conf(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 gyro config failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 gyro config complete!");
    }

    bmi088_res = bmi08g_set_int_config(&gyro_int_cfg, &bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 gyro interrupt config failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 gyro interrupt config complete!");
    }

    bmi088_res = bmi08a_init(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 acc initialization failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 acc initialization complete!");
    }

    bmi088_res = bmi08a_set_meas_conf(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 accel config failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 accel config complete!");
    }

    bmi088_res = bmi08a_set_power_mode(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 accel power mode config failed: %d", bmi088_res);
    } else {
        LOG_INFO("BMI088 accel power mode config complete!");
    }

    bmi088_res = bmi08a_set_int_config(&accel_int_cfg, &bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 accel interrupt config failed: %d", bmi088_res);
        Error_Handler();
    } else {
        LOG_INFO("BMI088 accel interrupt config complete!");
    }
}

/**
 * Init and configure the BMM350 peripheral
 *
 * On error jump to Error_Handler
 */
void PERIF_BMM350_Init() {
    /* Setup BMM350 */
    bmm350_intf.address = 0x14;
    bmm350_intf.hi2c = &hi2c2;

    bmm350.delay_us = &stm32_bmm350_delay_us;
    bmm350.write = &stm32_bmm350_write;
    bmm350.read = &stm32_bmm350_read;
    bmm350.intf_ptr = &bmm350_intf;
    int8_t bmm350_res = bmm350_init(&bmm350);
    if (bmm350_res != 0) {
        LOG_ERROR("BMM350 initialization failed: %d", bmm350_res);
        // Error_Handler();
    } else {
        LOG_INFO("BMM350 initialization complete!");
    }
    bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &bmm350);
    bmm350_set_powermode(BMM350_NORMAL_MODE, &bmm350);
}

/**
 * Init and configure the BNO055 connected to QWIIC
 *
 * On error jump to ErrorHandler
 */
void PERIF_BNO055_Init() {
    /* Setup BNO055 (on qwiic port) */
    bno055.bus_read = &stm32_bno055_bus_read;
    bno055.bus_write = &stm32_bno055_bus_write;
    bno055.delay_msec = &stm32_bno055_delay_us;
    bno055.dev_addr = 0x28;
    s8 bno055_res = bno055_init(&bno055);
    if (bno055_res != 0) {
        LOG_ERROR("BNO055 initialization failed: %d", bno055_res);
        Error_Handler();
    } else {
        LOG_INFO("BNO055 initialization complete!");
    }
}

/**
 * Init and configure the dynamixel servos. Ping
 * the servos to check if they are ready for use.
 * This setup requires DMA and notifications, so do this
 * only from a task.
 *
 * On error jump to Error_Handler
 */
void PERIF_Dynamixel_Init() {
    dynamixel_uart_context.huart = &huart6;
    dynamixel_uart_context.callerThread = osThreadGetId();

    if (dynamixel_uart_context.callerThread == NULL) {
        LOG_ERROR("dynamixel_uart_context.callerThread is NULL");
        Error_Handler();
    }

    DYNAMIXEL_ERROR_CHECK(
        dynamixel_bus_init(&dynamixel_bus, &dynamixel_read_uart_dma, &dynamixel_write_uart_dma, &dynamixel_uart_context
        ));
    int error_count = 0;
    for (int i = 0; i < 3 * 6; i++) {
        LOG_INFO("Configuring Servo %d...", i);

        DYNAMIXEL_ERROR_CHECK(dynamixel_init(&dynamixel_servo[i], i + 1, DYNAMIXEL_XL430, &dynamixel_bus));

        const dynamixel_error_t res = dynamixel_ping(&dynamixel_servo[i]);
        if (res != DYNAMIXEL_ERROR_NONE) {
            LOG_ERROR("dynamixel_ping failed: %d", res);
            error_count += 1;
        }
    }
    if (error_count > 0) {
        LOG_ERROR("Failed to initialize %d servos", error_count);
        Error_Handler();
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
    (void) argument;

    PERIF_Dynamixel_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    controller_ctx_t controller_ctx;
    controller_command_t cmd = {
        .velocity = 0,
        .heading = 0,
        .height = 100,
    };
    controller_attitude_t attitude = {
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
    };

    controller_init(&controller_ctx);
    controller_set_state_callback(&controller_ctx, stm32_state_change_cb, NULL);

    running_avg_t servo_read_ticks, servo_write_ticks, controller_update_ticks;

    running_avg_init(&servo_read_ticks);
    running_avg_init(&servo_write_ticks);
    running_avg_init(&controller_update_ticks);

    TickType_t t0, t1;
    int clock = 0;

    xEventGroupSetBits(systemEventsHandle, EVT_CONTROLLER_READY);

    /* Infinite loop */
    for (;;) {
        clock++;

        cmd.velocity = updated_velocity;
        cmd.heading = updated_heading;
        cmd.height = updated_height;

        t0 = xTaskGetTickCount();
        // Determine the actual servo positions
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];
            const struct leg *current_leg = &controller_ctx.cfg->leg[i];

            dynamixel_servo_t leg_servos[3] = {
                dynamixel_servo[current_leg->servos[0] - 1],
                dynamixel_servo[current_leg->servos[1] - 1],
                dynamixel_servo[current_leg->servos[2] - 1],
            };
            float32_t measured_leg_servo_angles[3];

            if (read_actual_servo_position(leg_servos, 3, measured_leg_servo_angles) < 0) {
                // LOG_WARN("Failed to read servo position for leg %d\r\n", i);
                // Use the defined angles as a stop gap
                // FIXME, these angles are uncompensated
                arm_vec_copy_f32(controller_ctx.robot.leg_state[i].next_joint_angles,
                                 controller_ctx.robot.leg_state[i].actual_joint_angles, 3);
                continue;
            }

            // Compensate angles for geometry
            if (controller_ctx.state == CTRL_SYNCING || controller_ctx.state == CTRL_BOOT || controller_ctx.state == CTRL_POWERDOWN) {
                // We exclusively use the measured position
                current_leg_state->actual_joint_angles[0] = measured_leg_servo_angles[0];
                current_leg_state->actual_joint_angles[1] = -measured_leg_servo_angles[1];
                current_leg_state->actual_joint_angles[2] = measured_leg_servo_angles[2] + D2R(25);
            } else {
                // We use a mix of the calculated angle and the measured angle to offset any measurement error
                // and compensate for a bit of deadzone at low speeds
                // Use alpha to tune the mix
                float32_t compensated_angles[3] = {
                    measured_leg_servo_angles[0],
                    -measured_leg_servo_angles[1],
                    measured_leg_servo_angles[2] + D2R(25)
                };
                const float32_t alpha = 0.8f;
                current_leg_state->actual_joint_angles[0] = compensated_angles[0] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[0];
                current_leg_state->actual_joint_angles[1] = compensated_angles[1] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[1];
                current_leg_state->actual_joint_angles[2] = compensated_angles[2] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[2];
            }
        }
        t1 = xTaskGetTickCount();
        running_avg_add(&servo_read_ticks, t1-t0);

        t0 = xTaskGetTickCount();
        controller_update(&controller_ctx, &attitude, &cmd, MAIN_LOOP_INTERVAL / 1000);
        t1 = xTaskGetTickCount();
        running_avg_add(&controller_update_ticks, t1-t0);

        t0 = xTaskGetTickCount();
        // Write next values to the servos
        if (controller_ctx.state != CTRL_POWERDOWN) {
            for (int i = 0; i < 6; i++) {
                struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];
                const struct leg *current_leg = &controller_ctx.cfg->leg[i];

                dynamixel_servo_t leg_servos[3] = {
                    dynamixel_servo[current_leg->servos[0] - 1],
                    dynamixel_servo[current_leg->servos[1] - 1],
                    dynamixel_servo[current_leg->servos[2] - 1],
                };
                float32_t leg_servo_angles[3];

                // Compensate angles for geometry
                leg_servo_angles[0] = current_leg_state->next_joint_angles[0];
                leg_servo_angles[1] = -current_leg_state->next_joint_angles[1];
                leg_servo_angles[2] = current_leg_state->next_joint_angles[2] - D2R(25);

                uint8_t limit_alert = 0;
                for (int axis = 0; axis < 3; axis++) {
                    if (leg_servo_angles[axis] < current_leg->limits[axis][0] || leg_servo_angles[axis] > current_leg->limits[axis][1]) {
                        LOG_ERROR("Limit alert triggered, leg %d, axis %d", i, axis);
                        LOG_ERROR("Calculated value %5.2f, limits %5.2f, %5.2f", leg_servo_angles[axis], current_leg->limits[axis][0], current_leg->limits[axis][1]);
                        limit_alert = 1;
                    }
                }

                if (limit_alert && controller_ctx.state == CTRL_WALKING) {
                    cmd.velocity = 0.0f;
                    controller_ctx.next_state = CTRL_POWERDOWN;
                    continue;
                }

                write_next_servo_position(leg_servos, 3, leg_servo_angles);
            }
        }
        t1 = xTaskGetTickCount();
        running_avg_add(&servo_write_ticks, t1-t0);

        // FIXME clock is a horrible way to print data periodically, do better.
        if (clock % (5 * 5) == 0) {
            LOG_INFO("[GYR] x %5.2f, y %5.2f, z %5.2f",
                gyro_snapshot[0], gyro_snapshot[1], gyro_snapshot[2]);
            LOG_INFO("[ACC] x %5.2f, y %5.2f, z %5.2f",
                accel_snapshot[0], accel_snapshot[1], accel_snapshot[2]);
            LOG_INFO("[MAG] x %.8f, y %.8f, z %.8f",
                mag_snapshot[0], mag_snapshot[1], mag_snapshot[2]);
            LOG_INFO("[EKF] roll %5.2f, pitch %5.2f, yaw %5.2f",
                ekf_out.roll, ekf_out.pitch, ekf_out.yaw);
        }

        if (clock % (5 * 30) == 0) {
            LOG_INFO("Average read ticks %ld, write ticks %ld, controller ticks %ld",
                running_avg_get(&servo_read_ticks),
                running_avg_get(&servo_write_ticks),
                running_avg_get(&controller_update_ticks)
            );
        }

        // Schedule at fixed 5 Hz
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_LOOP_INTERVAL));
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSpiSlaveTask */
/**
* @brief Function implementing the spiSlaveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpiSlaveTask */
void StartSpiSlaveTask(void *argument)
{
  /* USER CODE BEGIN StartSpiSlaveTask */
    UNUSED(argument);

    // Wait for the controller to become ready
    xEventGroupWaitBits(systemEventsHandle,
                EVT_CONTROLLER_READY,
                pdFALSE,
                pdTRUE,
                portMAX_DELAY);

    // Message format (7 bytes)
    //   uint8_t magic
    //   uint8_t reserved
    //   uint8_t register
    //   uint32_t value
    uint8_t buffer[16];

    /* Infinite loop */
    for (;;) {
        LOG_INFO("[StartSpiSlaveTask] Waiting for receive...");

        // Start length byte reception
        if (HAL_SPI_Receive_IT(&hspi1, buffer, 7) != HAL_OK) {
            LOG_ERROR("[StartSpiSlaveTask] HAL_SPI_Receive error");

            // Wait, reset and try again
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const uint32_t flags = osThreadFlagsWait(SPI1_RX_CPLT | SPI1_ERROR, osFlagsWaitAny, portMAX_DELAY);
        if (flags == (uint32_t) osErrorTimeout) {
            LOG_DEBUG("[StartSpiSlaveTask] osThreadFlagsWait timeout");
            continue;
        }

        if (flags & (1U << 31)) {
            LOG_DEBUG("[StartSpiSlaveTask] osThreadFlagsWait error %ld", flags);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (flags == SPI1_ERROR) {
            LOG_DEBUG("[StartSpiSlaveTask] receive error");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check the magic header
        if (buffer[0] != 0xA5) {
            LOG_ERROR("[StartSpiSlaveTask] Invalid magic header: 0x%02x", buffer[0]);
            continue;
        }

        // Received the data
        switch (buffer[2]) {
            case 0x01: {
                // Command set speed
                const float32_t *new_velocity = (float32_t *) &buffer[3];
                if (*new_velocity < 0 || *new_velocity > 100) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new velocity %5.2f", *new_velocity);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set speed to %5.2f mm/s", *new_velocity);
                updated_velocity = *new_velocity;
                break;
            }

            case 0x02: {
                // Command set heading
                const float32_t *new_heading = (float32_t *) &buffer[3];
                if (*new_heading < 0 || *new_heading > M_PI) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new heading %5.3f rad", *new_heading);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set heading to %5.3f rad", *new_heading);
                updated_heading = *new_heading;
                break;
            }

            case 0x03: {
                // Command set height
                const float32_t *new_height = (float32_t *) &buffer[3];
                if (*new_height < 50 || *new_height > 170) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new body height %5.2f", *new_height);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set new body height to %5.2f mm", *new_height);
                updated_height = *new_height;
                break;
            }

            default:
                LOG_WARN("[StartSpiSlaveTask] Unknown command: 0x%02x", buffer[2]);
                break;
        }
    }
  /* USER CODE END StartSpiSlaveTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */
    UNUSED(argument);

    // Wait for the EKF to become ready
    xEventGroupWaitBits(systemEventsHandle,
                EVT_EKF_READY,
                pdFALSE,
                pdTRUE,
                portMAX_DELAY);

    struct bmi08_gyro_int_channel_cfg gyro_int_cfg = {
        .int_channel = BMI08_INT_CHANNEL_3,
        .int_type = BMI08_GYRO_INT_DATA_RDY,
        .int_pin_cfg = {
            BMI08_INT_ACTIVE_LOW,
            BMI08_INT_MODE_PUSH_PULL,
            BMI08_ENABLE
        }
    };

    if (bmi08g_set_int_config(&gyro_int_cfg, &bmi088) < 0) {
        LOG_ERROR("[GyroTask] Failed to set interrupt config");
        vTaskDelete(NULL);
        return;
    }

    sensor_sample_t sample;
    struct bmi08_sensor_data gyro;
    uint8_t range = bmi088.gyro_cfg.range;
    float32_t sensor_frame_data[3];

    gyro_interrupt_enable = 1;

  /* Infinite loop */
    for (;;) {
        // Wait for interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Only one task can use SPI at a time
        if (xSemaphoreTake(spiMutexHandle, (TickType_t) 1) != pdTRUE) {
            continue;
        }

        // Read BMI088 gyro (SPI)
        if (bmi08g_get_data(&gyro, &bmi088) != 0) {
            LOG_ERROR("[GyroTask] Failed to get data");
            continue;
        }

        xSemaphoreGive(spiMutexHandle);

        // Scale and convert to rad/s
        sensor_frame_data[0] = stm32_bmi08g_scale_data(gyro.x, range) * (float32_t)(M_PI / 180.0f);
        sensor_frame_data[1] = stm32_bmi08g_scale_data(gyro.y, range) * (float32_t)(M_PI / 180.0f);
        sensor_frame_data[2] = stm32_bmi08g_scale_data(gyro.z, range) * (float32_t)(M_PI / 180.0f);
        stm32_bmi08g_sensor_to_ned(sensor_frame_data, sample.data);

        sample.type = SENSOR_GYRO;
        sample.tick = xTaskGetTickCount();

        // Non-blocking send (gyro is high rate)
        xQueueSendToBack(ekf_queueHandle, &sample, 0);
    }
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartAccelTask */
/**
* @brief Function implementing the accelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccelTask */
void StartAccelTask(void *argument)
{
  /* USER CODE BEGIN StartAccelTask */
    UNUSED(argument);

    // Wait for the EKF to become ready
    xEventGroupWaitBits(systemEventsHandle,
                EVT_EKF_READY,
                pdFALSE,
                pdTRUE,
                portMAX_DELAY);

    // Enable the accelerometer interrupt
    struct bmi08_accel_int_channel_cfg accel_int_cfg = {
        .int_channel = BMI08_INT_CHANNEL_1,
        .int_type = BMI08_ACCEL_INT_DATA_RDY,
        .int_pin_cfg = {
            BMI08_INT_ACTIVE_LOW,
            BMI08_INT_MODE_PUSH_PULL,
            BMI08_ENABLE
        }
    };

    if (bmi08a_set_int_config(&accel_int_cfg, &bmi088) < 0) {
        LOG_ERROR("[AccelTask] Failed to set interrupt config");
        vTaskDelete(NULL);
        return;
    }

    sensor_sample_t sample;
    struct bmi08_sensor_data accel;


    accel_interrupt_enable = 1;

    uint8_t range = bmi088.accel_cfg.range;
    float32_t sensor_frame_data[3];

    /* Infinite loop */
    for (;;) {
        // Wait for interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Only one task can use SPI at a time
        if (xSemaphoreTake(spiMutexHandle, (TickType_t) 5) != pdTRUE) {
            continue;
        }

        // Read BMI088 gyro (SPI)
        if (bmi08a_get_data(&accel, &bmi088) != 0) {
            LOG_ERROR("[AccelTask] Failed to get data");
            continue;
        }

        xSemaphoreGive(spiMutexHandle);

        // Scale and convert to m/s2
        sensor_frame_data[0] = stm32_bmi08a_scale_data(accel.x, range) * GRAVITY / 1000;
        sensor_frame_data[1] = stm32_bmi08a_scale_data(accel.y, range) * GRAVITY / 1000;
        sensor_frame_data[2] = stm32_bmi08a_scale_data(accel.z, range) * GRAVITY / 1000;
        stm32_bmi08a_sensor_to_ned(sensor_frame_data, sample.data);

        sample.type = SENSOR_ACCEL;
        sample.tick = xTaskGetTickCount();

        // Blocking send (accel is not that high rate)
        xQueueSendToBack(ekf_queueHandle, &sample, 5);
    }
  /* USER CODE END StartAccelTask */
}

/* USER CODE BEGIN Header_StartMagTask */
/**
* @brief Function implementing the magTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMagTask */
void StartMagTask(void *argument)
{
  /* USER CODE BEGIN StartMagTask */
    UNUSED(argument);

    // Wait for the EKF to become ready
    xEventGroupWaitBits(systemEventsHandle,
                EVT_EKF_READY,
                pdFALSE,
                pdTRUE,
                portMAX_DELAY);

    sensor_sample_t sample;
    struct bmm350_mag_temp_data data;
    float32_t sensor_frame_data[3];

    TickType_t xLastWakeTime = xTaskGetTickCount();

    float32_t bias[3] = { +37, -24, +35 };

    /* Infinite loop */
    for(;;) {
        if (bmm350_get_compensated_mag_xyz_temp_data(&data, &bmm350) < 0) {
            LOG_ERROR("[MagTask] Failed to get data");
        } else {
            // Convert sensor frame to NED
            // Values reported in uT (micro Tesla)
            sensor_frame_data[0] = data.x;
            sensor_frame_data[1] = data.y;
            sensor_frame_data[2] = data.z;

            stm32_bmm350_sensor_to_ned(sensor_frame_data, sample.data);

            // apply bias
            sample.data[0] = sample.data[0] - bias[0];
            sample.data[1] = sample.data[1] - bias[1];
            sample.data[2] = sample.data[2] - bias[2];

            sample.type = SENSOR_MAG;

            // Blocking send (mag is not low rate)
            xQueueSendToBack(ekf_queueHandle, &sample, 10);
        }

        // Schedule at 5 Hz
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / 5));
    }
  /* USER CODE END StartMagTask */
}

/* USER CODE BEGIN Header_StartEkfTask */
/**
* @brief Function implementing the ekfTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEkfTask */
void StartEkfTask(void *argument)
{
  /* USER CODE BEGIN StartEkfTask */
    UNUSED(argument);

    attitude_ekf_t ekf;
    sensor_sample_t sample;
    TickType_t last_gyro_tick = 0;

    attitude_ekf_init(&ekf);

    // Wait for the controller to become ready
    xEventGroupWaitBits(systemEventsHandle,
                EVT_CONTROLLER_READY,
                pdFALSE,
                pdTRUE,
                portMAX_DELAY);

    // Signal ready
    xEventGroupSetBits(systemEventsHandle, EVT_EKF_READY);

    for (;;) {
        if (xQueueReceive(ekf_queueHandle, &sample, portMAX_DELAY)) {

            if (sample.type == SENSOR_GYRO) {
                float32_t dt = (sample.tick - last_gyro_tick) * portTICK_PERIOD_MS * 0.001f;
                last_gyro_tick = sample.tick;

                attitude_ekf_predict(&ekf, sample.data, dt);

                gyro_snapshot[0] = sample.data[0];
                gyro_snapshot[1] = sample.data[1];
                gyro_snapshot[2] = sample.data[2];
            }
            else if (sample.type == SENSOR_ACCEL) {
                float32_t roll, pitch;
                compute_roll_pitch(sample.data, &roll, &pitch);

                attitude_ekf_update_accel(&ekf, roll, pitch);

                // Update snapshot for usage in mag
                accel_snapshot[0] = sample.data[0];
                accel_snapshot[1] = sample.data[1];
                accel_snapshot[2] = sample.data[2];
            }
            else if (sample.type == SENSOR_MAG) {
                float32_t mag_n[3];
                arm_vec_normalize_f32(sample.data, mag_n, 3);

                float32_t yaw = compute_yaw_from_mag(accel_snapshot, mag_n);

                attitude_ekf_update_mag(&ekf, yaw);

                mag_snapshot[0] = sample.data[0];
                mag_snapshot[1] = sample.data[1];
                mag_snapshot[2] = sample.data[2];
            }

            // Update with our latest state
            ekf_out.roll = ekf.x[0];
            ekf_out.pitch = ekf.x[1];
            ekf_out.yaw = ekf.x[2];
            ekf_out.gyro_bias[0] = ekf.x[3];
            ekf_out.gyro_bias[1] = ekf.x[4];
            ekf_out.gyro_bias[2] = ekf.x[5];
            ekf_out.last_update_ts = sample.tick;
        }
    }
  /* USER CODE END StartEkfTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
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

    /* Set indicator LED to red */
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_SET);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
