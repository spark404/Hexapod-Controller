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
#include <assert.h>
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

#include "ring_buffer.h"
#include "log_sink_port.h"
#include "cmd_link.h"
#include "periodic.h"
#include "accel_gate.h"
#include "command_adapter.h"

#include "dynamixel/dynamixel.h"

#include "hexapodmath/additional_functions.h"

#include "robot.h"
#include "calculator.h"
#include "servos.h"
#include "servo_profile.h"
#include "log.h"
#include "bus_diag.h"
#include "dynamixel/protocol.h"
#include "controller.h"
#include "controller_config.h"
#include "controller_math.h"
#include "error_handling.h"
#include "event_groups.h"
#include "rgb_led.h"
#include "robot_config.h"
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

typedef struct {
    uint8_t *data;
    uint16_t len;
} tx_msg_t;


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

#define HZ_TO_INTERVAL(hz) (1000 / (uint32_t)(hz))
#define MAIN_LOOP_INTERVAL HZ_TO_INTERVAL(1)

// Periodic reports of the default task. Reports with the same start and a period that is a
// multiple of another one fall together: the status block (servo bus, sensors) every
// STATUS_REPORT_PERIOD_S, the stack usage with every tenth block.
#define STATUS_REPORT_PERIOD_S 30
#define LOAD_REPORT_PERIOD_S 5      // debug level only
#define STACK_REPORT_PERIOD_S 300
#define STACK_LOW_PERCENT 20        // a task with less stack than this free is reported as a warning
#define HEAP_LOW_BYTES 4096         // the lowest the FreeRTOS heap has been, below this it is a warning
#define SERVO_VOLTAGE_LOW_DV 95     // servo input below 9.5 V (units of 0.1 V) is reported as a warning
#define SERVO_TEMPERATURE_HIGH_C 65 // servo temperature at or above this is reported as a warning
#define CONTROL_LOOP_INTERVAL HZ_TO_INTERVAL(10)
#define SERVO_LOOP_INTERVAL HZ_TO_INTERVAL(50)
#define MAG_LOOP_INTERVAL HZ_TO_INTERVAL(5)

#define DMA_RX_BUF_SIZE 1024 // Can handle 2.5 ms of data at 4 Mbps
#define RX_RING_SIZE 4096    // Enough space to handle driver delay

#define RX_DMA_TC 0x1
#define RX_DMA_HT 0x2
#define RX_DMA_IDLE 0x4
#define RX_DMA_ERROR 0x8
#define TX_DMA_TC 0x01

// The servos interpolate between the goals, with a profile written with every goal:
//  FIXED    the same profile for every goal (T4.1)
//  COMPUTED per joint, sized to cover the move in about one control period (T4.2)
#define SERVO_PROFILE_MODE_FIXED    0
#define SERVO_PROFILE_MODE_COMPUTED 1
#define SERVO_PROFILE_MODE          SERVO_PROFILE_MODE_COMPUTED

// Fixed profile: FIXED mode, and the first goals after a sync
#define SERVO_PROFILE_VELOCITY      4.0f    // rad/s
#define SERVO_PROFILE_ACCELERATION  40.0f   // rad/s²

// Computed profile: T = factor x the duration of the targets, ramps of fraction x T.
// T a little longer than the period: the next goal arrives in the last ramp, never
// after a stop (bench E4 tested 0.8, 1.2 and 1.5).
#define SERVO_PROFILE_T_FACTOR      1.2f
#define SERVO_PROFILE_TR_FRACTION   0.25f

// Blend rate for merging actual measurements into state
#define ALPHA 1.0f

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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spiSlaveTask */
osThreadId_t spiSlaveTaskHandle;
const osThreadAttr_t spiSlaveTask_attributes = {
  .name = "spiSlaveTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for magTask */
osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
  .name = "magTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ekfTask */
osThreadId_t ekfTaskHandle;
const osThreadAttr_t ekfTask_attributes = {
  .name = "ekfTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for usartRxTask */
osThreadId_t usartRxTaskHandle;
const osThreadAttr_t usartRxTask_attributes = {
  .name = "usartRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for usartTxTask */
osThreadId_t usartTxTaskHandle;
const osThreadAttr_t usartTxTask_attributes = {
  .name = "usartTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 1600 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for rgbLedTask */
osThreadId_t rgbLedTaskHandle;
const osThreadAttr_t rgbLedTask_attributes = {
  .name = "rgbLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ekf_queue */
osMessageQueueId_t ekf_queueHandle;
const osMessageQueueAttr_t ekf_queue_attributes = {
  .name = "ekf_queue"
};
/* Definitions for usart_tx_queue */
osMessageQueueId_t usart_tx_queueHandle;
const osMessageQueueAttr_t usart_tx_queue_attributes = {
  .name = "usart_tx_queue"
};
/* Definitions for rgb_led_queue */
osMessageQueueId_t rgb_led_queueHandle;
const osMessageQueueAttr_t rgb_led_queue_attributes = {
  .name = "rgb_led_queue"
};
/* Definitions for spiMutex */
osMutexId_t spiMutexHandle;
const osMutexAttr_t spiMutex_attributes = {
  .name = "spiMutex"
};
/* Definitions for usartMutex */
osMutexId_t usartMutexHandle;
const osMutexAttr_t usartMutex_attributes = {
  .name = "usartMutex"
};
/* Definitions for uart_dma_mutex */
osMutexId_t uart_dma_mutexHandle;
const osMutexAttr_t uart_dma_mutex_attributes = {
  .name = "uart_dma_mutex"
};
/* Definitions for usart_rx_sem */
osSemaphoreId_t usart_rx_semHandle;
const osSemaphoreAttr_t usart_rx_sem_attributes = {
  .name = "usart_rx_sem"
};
/* Definitions for usart_tx_sem */
osSemaphoreId_t usart_tx_semHandle;
const osSemaphoreAttr_t usart_tx_sem_attributes = {
  .name = "usart_tx_sem"
};
/* Definitions for systemEvents */
osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};
/* USER CODE BEGIN PV */
/* Set when the HSE did not start and the clock fell back to the HSI */
volatile uint8_t g_clock_hse_failed = 0;
i2c_intf_ptr bmm350_intf;
bmm350_t bmm350;

spi_intf_ptr bmi088_acc_intf;
spi_intf_ptr bmi088_gyr_intf;
bmi08_t bmi088;

i2c_intf_ptr bno055_intf;
bno055_tt bno055;

dynamixel_bus_t dynamixel_bus;
dynamixel_servo_t dynamixel_servos[3 * 6];

volatile osThreadId_t servoCallbackThreadId;

// Lock of the command link state shared by the SPI task and the control task.
// A critical section of a few microseconds, usable from both tasks.
static uint32_t cmd_link_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void cmd_link_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static const cmd_link_port_t cmd_link_port = {cmd_link_lock, cmd_link_unlock};

float32_t velocity = 0.0f; // mm/s
float32_t heading = 0.0f; // rad
float32_t height = 100.f; // mm, body height from ground

ekf_output_t ekf_out;

uint8_t gyro_interrupt_enable = 0;
uint8_t accel_interrupt_enable = 0;

float32_t gyro_snapshot[3];
float32_t accel_snapshot[3];
float32_t mag_snapshot[3];
accel_gate_t accel_gate;   // written by the accel task, read by the status report

// USART buffers
static uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
static uint8_t rx_ring_buffer[RX_RING_SIZE];
static ringbuf_t rx_ring;
static size_t dma_rx_read_idx = 0;

servo_shared_state_t servo_shared_state;

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
void StartUsartRxTask(void *argument);
void StartUsartTxTask(void *argument);
void StartControlTask(void *argument);
void StartServoTask(void *argument);
void rgb_led_task(void *argument);

/* USER CODE BEGIN PFP */
static void report_bus(void);
static void report_sensors(void);
static void report_accel_gate(void);
static void report_servo_health(void);
static void report_load(void);
static void report_stacks(void);
static void rtos_check_created(void);
static int clock_hse_fallback(void);
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

ssize_t usart_read(uint8_t *dst, size_t len, uint32_t timeout);
ssize_t usart_write(const uint8_t *src, size_t len, uint32_t timeout);

ssize_t dynamixel_read_uart_dma(uint8_t *rxBuffer, size_t size, void *pvContext);
ssize_t dynamixel_write_uart_dma(const uint8_t *txBuffer, size_t size, void *pvContext);

void PERIF_BMI088_Init(void);
void PERIF_BMM350_Init(void);
void PERIF_BNO055_Init(void);
void PERIF_Dynamixel_Init(void);
void PERIF_Dynamixel_Configure(void);

static float32_t stm32_bmi08a_scale_data(int16_t raw, uint8_t accel_range);
static float32_t stm32_bmi08g_scale_data(int16_t raw, uint8_t gyro_range);
static void stm32_bmi08a_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]);
static void stm32_bmi08g_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]);
static void stm32_bmm350_sensor_to_ned(const float32_t sensor[3], float32_t ned[3]);

void dynamixel_flush_uart_dma(void *pvContext);

static size_t dma_rx_write_idx(void);
static void uart_rx_drain_dma(void);
static void uart_rx_flush_dma(void);
static void uart_rx_reset_dma(void);

static void stm32_state_change_cb(controller_ctx_t *ctx, controller_state_t from, controller_state_t to, void *user_data);
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

    // From here printf() queues into a buffer drained by DMA instead of busy-waiting
    log_sink_uart_init();
    cmd_link_init(&cmd_link_port);

    HAL_TIM_Base_Start(&htim1);

    // Free running cycle counter, used to time servo bus transactions.
    // htim1 can not be used for that, the sensor delay routines reset it.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    bus_diag_init();

    LOG_INFO("[Main] Hexapod Control Firmware");
    LOG_INFO("[Main] Build: %s %s", __DATE__, __TIME__);
    if (g_clock_hse_failed) {
        LOG_WARN("[Main] HSE failed to start, running from HSI (SYSCLK %lu Hz)", (unsigned long) SystemCoreClock);
    } else {
        LOG_INFO("[Main] Clock: HSE, SYSCLK %lu Hz", (unsigned long) SystemCoreClock);
    }

    LOG_INFO("[Main] Core initialisation");
    ringbuf_init(&rx_ring, rx_ring_buffer, sizeof(rx_ring_buffer));

    LOG_INFO("[Main] Peripheral initialisation");
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_RESET);

    PERIF_BMI088_Init();
    PERIF_BMM350_Init();

    // Connected via QWIIC
    // PERIF_BNO055_Init();

    LOG_INFO("[Main] Initialisation complete");
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of spiMutex */
  spiMutexHandle = osMutexNew(&spiMutex_attributes);

  /* creation of usartMutex */
  usartMutexHandle = osMutexNew(&usartMutex_attributes);

  /* creation of uart_dma_mutex */
  uart_dma_mutexHandle = osMutexNew(&uart_dma_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of usart_rx_sem */
  usart_rx_semHandle = osSemaphoreNew(4096, 0, &usart_rx_sem_attributes);

  /* creation of usart_tx_sem */
  usart_tx_semHandle = osSemaphoreNew(4096, 0, &usart_tx_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ekf_queue */
  ekf_queueHandle = osMessageQueueNew (32, sizeof(sensor_sample_t), &ekf_queue_attributes);

  /* creation of usart_tx_queue */
  usart_tx_queueHandle = osMessageQueueNew (16, sizeof(tx_msg_t), &usart_tx_queue_attributes);

  /* creation of rgb_led_queue */
  rgb_led_queueHandle = osMessageQueueNew (8, sizeof(rgb_led_cmd_t), &rgb_led_queue_attributes);

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

  /* creation of usartRxTask */
  usartRxTaskHandle = osThreadNew(StartUsartRxTask, (void*) &huart6, &usartRxTask_attributes);

  /* creation of usartTxTask */
  usartTxTaskHandle = osThreadNew(StartUsartTxTask, (void*) &huart6, &usartTxTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);

  /* creation of rgbLedTask */
  rgbLedTaskHandle = osThreadNew(rgb_led_task, NULL, &rgbLedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* creation of systemEvents */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    rtos_check_created();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim1.Init.Prescaler = 84-1;
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
  huart6.Init.BaudRate = 2000000;
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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, SPI2_CS_ACC_Pin|SPI2_CS_GYR_Pin|ST_LED_R_Pin|ST_LED_G_Pin
                          |ST_LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI2_CS_ACC_Pin SPI2_CS_GYR_Pin ST_LED_R_Pin ST_LED_G_Pin
                           ST_LED_B_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_ACC_Pin|SPI2_CS_GYR_Pin|ST_LED_R_Pin|ST_LED_G_Pin
                          |ST_LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_INT_ACC_Pin SPI2_INT_GYR_Pin */
  GPIO_InitStruct.Pin = SPI2_INT_ACC_Pin|SPI2_INT_GYR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

// Servo bus diagnostics of the last STATUS_REPORT_PERIOD_S: one line, or the failures
// spelled out at warning level when there were any (see bus_diag.c)
static void report_bus(void)
{
    static bus_diag_snapshot_t prev;
    static bus_diag_snapshot_t cur;
    static bus_diag_report_t report;
    static bool started;

    if (!started) {
        // The first window starts here, not at boot
        bus_diag_snapshot(&prev, true);
        started = true;
    }

    bus_diag_snapshot(&cur, true);
    bus_diag_format(&prev, &cur, STATUS_REPORT_PERIOD_S, &report);
    prev = cur;

    if (report.window_errors) {
        LOG_WARN("%s", report.summary);
        LOG_WARN("%s", report.protocol);
        LOG_WARN("%s", report.transport);
        if (report.servos[0] != '\0') {
            LOG_WARN("%s", report.servos);
        }
    } else {
        LOG_INFO("%s", report.summary);
    }
    if (report.boot[0] != '\0') {
        LOG_INFO("%s", report.boot);
    }
}

// Latest sensor values and the attitude estimate, one line
static void report_sensors(void)
{
    LOG_INFO("[Sensors] gyr %5.2f %5.2f %5.2f | acc %5.2f %5.2f %5.2f | mag %.2f %.2f %.2f | "
             "ekf roll %5.2f pitch %5.2f yaw %5.2f",
             gyro_snapshot[0], gyro_snapshot[1], gyro_snapshot[2],
             accel_snapshot[0], accel_snapshot[1], accel_snapshot[2],
             mag_snapshot[0], mag_snapshot[1], mag_snapshot[2],
             ekf_out.roll, ekf_out.pitch, ekf_out.yaw);
}

// What the accelerometer check rejected since the last report: samples whose length is not 1 g
static void report_accel_gate(void)
{
    static uint32_t prev_accepted, prev_rejected;
    const uint32_t accepted = accel_gate.accepted, rejected = accel_gate.rejected;
    const uint32_t d_ok = accepted - prev_accepted, d_bad = rejected - prev_rejected;
    prev_accepted = accepted;
    prev_rejected = rejected;

    if (d_bad == 0) {
        LOG_INFO("[Accel] %lu samples, none rejected", (unsigned long) d_ok);
        return;
    }
    LOG_INFO("[Accel] %lu of %lu samples rejected (length not 1 g +-%.1f m/s2), last bad raw %d %d %d, %.1f m/s2",
             (unsigned long) d_bad, (unsigned long) (d_ok + d_bad), (double) accel_gate.tolerance,
             accel_gate.last_rejected_raw[0], accel_gate.last_rejected_raw[1], accel_gate.last_rejected_raw[2],
             (double) accel_gate.last_rejected_norm);
}

// Input voltage and temperature of the servos (read with every state read): the supply sagging or a servo
// heating up would show here before it shows as a missing reply
static void report_servo_health(void)
{
    uint16_t volt[6][3];
    uint8_t temp[6][3];
    servo_copy_health(volt, temp, &servo_shared_state);

    unsigned vmin = 0xFFFF, vmax = 0, tmax = 0, vmin_i = 0, tmax_i = 0;
    for (unsigned i = 0; i < 18; i++) {
        const unsigned v = volt[i / 3][i % 3], t = temp[i / 3][i % 3];
        if (v == 0) {
            continue;                                          // not read yet
        }
        if (v < vmin) { vmin = v; vmin_i = i; }
        if (v > vmax) { vmax = v; }
        if (t > tmax) { tmax = t; tmax_i = i; }
    }
    if (vmax == 0) {
        return;
    }

    // XL430: 6.5 to 12 V, the temperature limit is 80 C
    const bool low = vmin < SERVO_VOLTAGE_LOW_DV || tmax >= SERVO_TEMPERATURE_HIGH_C;
    if (low) {
        LOG_WARN("[Servos] input %u.%u to %u.%u V (lowest: servo %u), temperature up to %u C (servo %u): out of range",
                 vmin / 10, vmin % 10, vmax / 10, vmax % 10, vmin_i, tmax, tmax_i);
    } else {
        LOG_INFO("[Servos] input %u.%u to %u.%u V (lowest: servo %u), temperature up to %u C (servo %u)",
                 vmin / 10, vmin % 10, vmax / 10, vmax % 10, vmin_i, tmax, tmax_i);
    }
}

// Femur and tibia load of every leg, % of maximum, servo frame
static void report_load(void)
{
    float32_t load[6][3];
    servo_copy_load(load, &servo_shared_state);
    LOG_DEBUG("[LOAD] f/t %%: L0 %.1f/%.1f L1 %.1f/%.1f L2 %.1f/%.1f L3 %.1f/%.1f L4 %.1f/%.1f L5 %.1f/%.1f",
        load[0][1], load[0][2], load[1][1], load[1][2], load[2][1], load[2][2],
        load[3][1], load[3][2], load[4][1], load[4][2], load[5][1], load[5][2]);
}

// Free stack of every task in one line: a warning, with the low ones marked, when any
// task has less than STACK_LOW_PERCENT of its stack left
static void report_stacks(void)
{
    const osThreadId_t tasks[] = {
        defaultTaskHandle, spiSlaveTaskHandle, gyroTaskHandle,
        accelTaskHandle, magTaskHandle, ekfTaskHandle,
        usartRxTaskHandle, usartTxTaskHandle, controlTaskHandle,
        servoTaskHandle, rgbLedTaskHandle
    };
    const char *const names[] = {
        "Default", "SPI", "Gyro", "Accel", "Mag", "EKF",
        "RX", "TX", "Control", "Servo", "LED"
    };
    const uint32_t sizes[] = {
        defaultTask_attributes.stack_size, spiSlaveTask_attributes.stack_size,
        gyroTask_attributes.stack_size, accelTask_attributes.stack_size,
        magTask_attributes.stack_size, ekfTask_attributes.stack_size,
        usartRxTask_attributes.stack_size, usartTxTask_attributes.stack_size,
        controlTask_attributes.stack_size, servoTask_attributes.stack_size,
        rgbLedTask_attributes.stack_size
    };

    char line[256];
    size_t used = 0;
    bool low = false;

    for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
        if (tasks[i] == NULL || used >= sizeof(line)) {
            continue;
        }
        const uint32_t free_bytes = osThreadGetStackSpace(tasks[i]);
        const bool is_low = free_bytes * 100U < sizes[i] * STACK_LOW_PERCENT;
        low = low || is_low;

        const int n = snprintf(line + used, sizeof(line) - used, "%s%s %lu/%lu%s",
                               used == 0 ? "" : ", ", names[i],
                               (unsigned long) free_bytes, (unsigned long) sizes[i], is_low ? "!" : "");
        if (n < 0) {
            break;
        }
        used += (size_t) n;
    }

    if (low) {
        LOG_WARN("[Stack] free/total bytes, ! = under %d%% free: %s", STACK_LOW_PERCENT, line);
    } else {
        LOG_INFO("[Stack] free/total bytes: %s", line);
    }

    // Every task stack and RTOS object comes from the FreeRTOS heap; creating one fails quietly
    // when it runs out (the LED task was never created for that reason)
    const size_t heap_free = xPortGetFreeHeapSize();
    const size_t heap_low = xPortGetMinimumEverFreeHeapSize();
    if (heap_low < HEAP_LOW_BYTES) {
        LOG_WARN("[Heap] free %lu B, lowest ever %lu B of %lu B: running low",
                 (unsigned long) heap_free, (unsigned long) heap_low, (unsigned long) configTOTAL_HEAP_SIZE);
    } else {
        LOG_INFO("[Heap] free %lu B, lowest ever %lu B of %lu B",
                 (unsigned long) heap_free, (unsigned long) heap_low, (unsigned long) configTOTAL_HEAP_SIZE);
    }
}

// An RTOS object that could not be created is a NULL handle and nothing else: no error, no log.
// Called once everything has been created.
static void rtos_check_created(void)
{
    const struct {
        const char *name;
        const void *handle;
    } objects[] = {
        {"spiMutex", spiMutexHandle}, {"usartMutex", usartMutexHandle}, {"uart_dma_mutex", uart_dma_mutexHandle},
        {"usart_rx_sem", usart_rx_semHandle}, {"usart_tx_sem", usart_tx_semHandle},
        {"ekf_queue", ekf_queueHandle}, {"usart_tx_queue", usart_tx_queueHandle}, {"rgb_led_queue", rgb_led_queueHandle},
        {"defaultTask", defaultTaskHandle}, {"spiSlaveTask", spiSlaveTaskHandle}, {"gyroTask", gyroTaskHandle},
        {"accelTask", accelTaskHandle}, {"magTask", magTaskHandle}, {"ekfTask", ekfTaskHandle},
        {"usartRxTask", usartRxTaskHandle}, {"usartTxTask", usartTxTaskHandle}, {"controlTask", controlTaskHandle},
        {"servoTask", servoTaskHandle}, {"rgbLedTask", rgbLedTaskHandle}, {"systemEvents", systemEventsHandle},
    };

    for (size_t i = 0; i < sizeof(objects) / sizeof(objects[0]); i++) {
        if (objects[i].handle == NULL) {
            LOG_ERROR("[RTOS] %s was not created: FreeRTOS heap too small (free %lu B)", objects[i].name,
                      (unsigned long) xPortGetFreeHeapSize());
        }
    }
}
/**
 * HSE start-up fallback.
 *
 * SystemClock_Config() calls Error_Handler() when HAL_RCC_OscConfig() times
 * out waiting for the crystal. In that state HSEON is set, HSERDY is not, the
 * PLL is still off and SYSCLK is still the HSI. This switches the HSE off and
 * builds the same 84 MHz PLL output from the HSI (M=16, N=336, P=4, Q=7), so
 * the HAL_RCC_ClockConfig() call that follows in SystemClock_Config() succeeds.
 * Timing is then HSI-accurate (about 1 %) instead of crystal-accurate.
 *
 * Returns 1 when the fallback clock is running, 0 if the error was something
 * else or the fallback itself failed. Only one attempt is made.
 */
static int clock_hse_fallback(void)
{
    if (g_clock_hse_failed) {
        return 0;
    }
    if ((RCC->CR & RCC_CR_HSEON) == 0U || (RCC->CR & RCC_CR_HSERDY) != 0U ||
        (RCC->CR & RCC_CR_PLLON) != 0U) {
        return 0;
    }

    g_clock_hse_failed = 1;

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
    osc.HSEState = RCC_HSE_OFF;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;
    return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

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
    // TIM1 is clocked from PCLK2 (84 MHz); prescaler 84-1 gives a 1 MHz
    // counter (1 us per tick) and ARR 0xFFFF-1 caps the max delay at ~65 ms.
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < period);
}

BMI08_INTF_RET_TYPE stm32_bmi08_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL) {
        return -1;
    }

    spi_intf_ptr *spi_intf = (spi_intf_ptr *) intf_ptr;

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(spi_intf->hspi, &reg_addr, 1, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY) {}

    HAL_SPI_Receive(spi_intf->hspi, reg_data, len, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY) {}

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
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY) {}

    HAL_SPI_Transmit(spi_intf->hspi, (uint8_t *) reg_data, len, 50);
    while (HAL_SPI_GetState(spi_intf->hspi) == HAL_SPI_STATE_BUSY) {}

    HAL_GPIO_WritePin(spi_intf->CS_Port, spi_intf->CS_Pin, GPIO_PIN_SET);

    return 0;
}

void stm32_bmi08_delay_us(uint32_t period, void *intf_ptr) {
    (void) intf_ptr;
    // TIM1 is clocked from PCLK2 (84 MHz); prescaler 84-1 gives a 1 MHz
    // counter (1 us per tick) and ARR 0xFFFF-1 caps the max delay at ~65 ms.
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < period);
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

ssize_t usart_read(uint8_t *dst, const size_t len, const uint32_t timeout) {
    size_t count = 0;

    if (len == 0) {
        return 0;
    }

    /* Ensure only one reader */
    if (osMutexAcquire(usartMutexHandle, timeout) != osOK) {
        bus_diag_event(BUS_EV_RD_MUTEX_TMO, 1);
        return -1;
    }

    while (count < len) {
        taskENTER_CRITICAL();
        size_t avail = ringbuf_available(&rx_ring);
        if (avail > 0) {
            size_t to_copy = (avail > (len - count)) ? (len - count) : avail;
            count += ringbuf_read(&rx_ring, &dst[count], to_copy);
        }
        taskEXIT_CRITICAL();

        if (count >= len) break;

        // Wait for more data
        if (osSemaphoreAcquire(usart_rx_semHandle, timeout) != osOK) {
            break; // Timeout, return what we have
        }
    }

    osMutexRelease(usartMutexHandle);
    if (count < len) {
        bus_diag_event(count == 0 ? BUS_EV_RD_EMPTY : BUS_EV_RD_PARTIAL, 1);
    }
    return (ssize_t)count;
}

ssize_t usart_write(const uint8_t *src, const size_t len, const uint32_t timeout) {
    if (len == 0) {
        return 0;
    }

    /* Exclusive access */
    if (osMutexAcquire(usartMutexHandle, timeout) != osOK) {
        bus_diag_event(BUS_EV_WR_MUTEX_TMO, 1);
        return -1;
    }

    /* Prepare TX */
    tx_msg_t msg = {
        .data = (uint8_t *)src,
        .len = len,
    };

    /* Clear completion semaphore */
    osSemaphoreAcquire(usart_tx_semHandle, 0);

    if (osMessageQueuePut(usart_tx_queueHandle, &msg, 0, timeout) != osOK) {
        bus_diag_event(BUS_EV_WR_TXQ_FULL, 1);
        osMutexRelease(usartMutexHandle);
        return -1;
    }

    /* Wait for TX complete */
    if (osSemaphoreAcquire(usart_tx_semHandle, timeout) != osOK) {
        bus_diag_event(BUS_EV_WR_TX_TMO, 1);
        osMutexRelease(usartMutexHandle);
        return -1;
    }

    osMutexRelease(usartMutexHandle);
    return (ssize_t)len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (log_sink_uart_tx_cplt(huart)) {
        return;
    }

    if (huart != &huart6) {
        return;
    }

    // HAL invokes this from the UART TC interrupt (UART_EndTransmit_IT), so the
    // shift register is empty and the line is physically idle. Switch to the
    // receiver and rearm RX DMA before returning -- doing it here in IRQ
    // context guarantees no task-level preemption between line-idle and DMA-
    // armed, which would otherwise drop the start of the servo's reply.
    HAL_HalfDuplex_EnableReceiver(huart);
    HAL_UART_Receive_DMA(huart, dma_rx_buf, DMA_RX_BUF_SIZE);

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(usartTxTaskHandle, TX_DMA_TC);
}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        return;
    }

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(usartRxTaskHandle, RX_DMA_HT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        return;
    }

    uint32_t notify;
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        notify = RX_DMA_IDLE;
    } else {
        notify = RX_DMA_TC;
    }

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(usartRxTaskHandle, notify);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (log_sink_uart_error(huart)) {
        return;
    }

    if (huart != &huart6) {
        return;
    }

    const uint32_t err = huart->ErrorCode;
    if (err & HAL_UART_ERROR_ORE) bus_diag_event(BUS_EV_UART_ORE, 1);
    if (err & HAL_UART_ERROR_FE) bus_diag_event(BUS_EV_UART_FE, 1);
    if (err & HAL_UART_ERROR_NE) bus_diag_event(BUS_EV_UART_NE, 1);
    if (err & HAL_UART_ERROR_PE) bus_diag_event(BUS_EV_UART_PE, 1);
    if (err & HAL_UART_ERROR_DMA) bus_diag_event(BUS_EV_UART_DMA, 1);

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(usartRxTaskHandle, RX_DMA_ERROR);
}

// Callback called by HAL when SPI receive complete (in ISR context)
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(spiSlaveTaskHandle, SPI1_RX_CPLT);
}

// The SPI link transfers a command and a status frame at the same time
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(spiSlaveTaskHandle, SPI1_RX_CPLT);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    // osThreadFlagsSet internally handles context switching from ISR
    osThreadFlagsSet(spiSlaveTaskHandle, SPI1_ERROR);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    UNUSED(GPIO_Pin);

    switch (GPIO_Pin) {
        case SPI2_INT_GYR_Pin: {
            if (!gyro_interrupt_enable) {
                break;
            }
            // osThreadFlagsSet internally handles context switching from ISR
            osThreadFlagsSet(gyroTaskHandle, 0x01);
            break;
        }
        case SPI2_INT_ACC_Pin: {
            if (!accel_interrupt_enable) {
                break;
            }
            // osThreadFlagsSet internally handles context switching from ISR
            osThreadFlagsSet(accelTaskHandle, 0x01);
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

    switch (from) {
        case CTRL_POWERDOWN:
            servo_set_request_powerdown(&servo_shared_state, false);
            break;
        case CTRL_WALKING:
        case CTRL_ROTATING:
            servo_set_limit_alert_enabled(&servo_shared_state, false);
            break;
        default:
            break;
    }

    switch (to) {
        case CTRL_POWERDOWN:
            rgb_led_set_color(RGB_LED_COLOR_MAGENTA);
            rgb_led_blink(500, 0.5f);
            servo_set_request_powerdown(&servo_shared_state, true);
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
            servo_set_limit_alert_enabled(&servo_shared_state, true);
            break;
        case CTRL_ROTATING:
            rgb_led_set_color(RGB_LED_COLOR_YELLOW);
            rgb_led_blink(500, 0.5f);
            servo_set_limit_alert_enabled(&servo_shared_state, true);
            break;
        default:
            break;
    }
}

static size_t dma_rx_write_idx(void)
{
    return DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
}

static void uart_rx_drain_dma(void)
{
    bool pushed = false;
    uint32_t dropped = 0;
    size_t write = dma_rx_write_idx();

    taskENTER_CRITICAL();
    while (dma_rx_read_idx != write) {
        uint8_t b = dma_rx_buf[dma_rx_read_idx];
        dma_rx_read_idx = (dma_rx_read_idx + 1) % DMA_RX_BUF_SIZE;

        if (!ringbuf_push(&rx_ring, b)) {
            dropped++;
        }
        pushed = true;
    }
    taskEXIT_CRITICAL();

    if (dropped > 0) {
        bus_diag_event(BUS_EV_RX_DROP, dropped);
    }

    if (pushed) {
        osSemaphoreRelease(usart_rx_semHandle);
    }
}

static void uart_rx_flush_dma(void)
{
    taskENTER_CRITICAL();
    ringbuf_reset(&rx_ring);
    taskEXIT_CRITICAL();
}

static void uart_rx_reset_dma(void)
{
    // Protect DMA reconfiguration
    osMutexAcquire(uart_dma_mutexHandle, osWaitForever);

    // Stop DMA receive
    if (HAL_UART_DMAStop(&huart6) != HAL_OK) {
        LOG_ERROR("[UART] Failed to stop DMA receiver");
    }

    // Reset indices
    taskENTER_CRITICAL();
    dma_rx_read_idx = 0;
    huart6.hdmarx->Instance->NDTR = DMA_RX_BUF_SIZE;
    taskEXIT_CRITICAL();

    // Flush the dynamixel bus
    dynamixel_bus_flush(dynamixel_servos[0].bus);

    // Restart DMA receiver
    HAL_HalfDuplex_EnableReceiver(&huart6);
    if (HAL_UART_Receive_DMA(&huart6, dma_rx_buf, DMA_RX_BUF_SIZE) != HAL_OK) {
        LOG_ERROR("[UART] Failed to restart DMA receiver");
    }

    osMutexRelease(uart_dma_mutexHandle);
}

ssize_t dynamixel_read_uart_dma(
    uint8_t *rxBuffer,
    size_t size,
    void *pvContext) {
    UNUSED(pvContext);
    return usart_read(rxBuffer, size, pdMS_TO_TICKS(10));
}

ssize_t dynamixel_write_uart_dma(
    const uint8_t *txBuffer,
    size_t size,
    void *pvContext) {
    UNUSED(pvContext);
    return usart_write(txBuffer, size, pdMS_TO_TICKS(10));
}

void dynamixel_flush_uart_dma(void *pvContext) {
    UNUSED(pvContext);
    uart_rx_flush_dma();
}

/**
 * Initialize and configure the BMI088 chip
 * Deactivate interrupts for now
 *
 * On error jump to Error_Handler
 */
void PERIF_BMI088_Init(void) {
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
void PERIF_BMM350_Init(void) {
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
void PERIF_BNO055_Init(void) {
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
void PERIF_Dynamixel_Init(void) {
    DYNAMIXEL_ERROR_CHECK(
        dynamixel_bus_init(&dynamixel_bus, &dynamixel_read_uart_dma, &dynamixel_write_uart_dma, NULL, NULL
        ));
    int error_count = 0;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            const uint8_t id = r.leg[i].servos[j];
            const int index = i * 3 + j;
            LOG_INFO("Checking Servo %d, leg %d, joint %d...", id, i, j);

            DYNAMIXEL_ERROR_CHECK(dynamixel_init(&dynamixel_servos[index], id, DYNAMIXEL_XL430, &dynamixel_bus));

            const dynamixel_error_t res = dynamixel_ping(&dynamixel_servos[index]);
            if (res == STATUS_ALERT_FLAG) {
                LOG_ERROR("Servo %d hardware alert", id);
                uint8_t hardware_status;
                if (dynamixel_get_byte_parameter(&dynamixel_servos[index], XL430_CT_RAM_HARDWARE_ERR_STATUS, &hardware_status) != STATUS_OK) {
                    LOG_ERROR("Servo %d failed to read hardware status", id);
                } else {
                    LOG_ERROR("Servo %d hardware status: 0x%02x", id, hardware_status);
                }
                error_count++;
            } else if (res != DYNAMIXEL_ERROR_NONE) {
                LOG_ERROR("dynamixel_ping failed: %d", res);
                error_count += 1;
            }
        }
    }
    if (error_count > 0) {
        LOG_ERROR("Failed to initialize %d servos", error_count);
        Error_Handler();
    }
}

void PERIF_Dynamixel_Configure(void) {
    for (int i = 0; i < 3 * 6; i++) {
        dynamixel_set_led(&dynamixel_servos[i], 1);

        // Return Delay Time = 10 * 2us = 20us. Must be > the firmware's
        // TX-to-RX direction-switch latency (TC IRQ -> EnableReceiver ->
        // Receive_DMA), otherwise the servo's preamble starts arriving before
        // the receiver is armed and the reply is parsed as garbage
        // (DNM_LL_ERR / 68). Default is 250 (500us), previous code used 20 (40us)
        // and 0 (0us) which both proved too tight.
        uint8_t rdt;
        if (dynamixel_get_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_RETURN_DELAY_TIME, &rdt) == DNM_OK) {
            if (rdt != 10) {
                dynamixel_set_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_RETURN_DELAY_TIME, 10);
            }
        }

        // Set Operating Mode to Position Control (3)
        uint8_t op_mode;
        if (dynamixel_get_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_OPERATING_MODE, &op_mode) == DNM_OK) {
            if (op_mode != 3) {
                dynamixel_set_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_OPERATING_MODE, 3);
            }
        }

        // Configure Drive Mode (Normal, Forward, Time-based Profile)
        // Bit 2: 0 for Velocity-based Profile, 1 for Time-based Profile
        uint8_t drive_mode;
        if (dynamixel_get_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_DRIVE_MODE, &drive_mode) == DNM_OK) {
            if (drive_mode != 0) {
                dynamixel_set_byte_parameter(&dynamixel_servos[i], XL430_CT_EEP_DRIVE_MODE, 0);
            }
        }

        dynamixel_set_led(&dynamixel_servos[i], 0);
    }
}

/**
 * Microseconds since a DWT cycle counter reading taken earlier. Good for
 * intervals below the 32 bit wrap time (about 51 s at 84 MHz).
 */
static uint32_t diag_elapsed_us(const uint32_t start_cycles)
{
    return (DWT->CYCCNT - start_cycles) / (SystemCoreClock / 1000000U);
}

/* Present Load (126, 2 bytes), Present Velocity (128, 4), Present Position
 * (132, 4), Velocity Trajectory (136, 4), Position Trajectory (140, 4), Present Input Voltage
 * (144, 2, units of 0.1 V) and Present Temperature (146, 1, degrees C) are adjacent: one
 * 21 byte read per servo gets all seven. */
#define XL430_STATE_LEN 21
#define XL430_STATUS_ALERT 0x80
#define XL430_PULSE_RAD (2.0f * (float32_t) M_PI / 4096.0f)

/** What one read of the state of all servos returned. Indexed [leg][joint]. */
typedef struct {
    uint32_t position[6][3];    /* pulses */
    uint32_t profile_position[6][3];     /* Position Trajectory, pulses */
    float32_t load[6][3];       /* % of maximum, servo frame */
    float32_t velocity[6][3];   /* rad/s, servo frame */
    float32_t tracking_error[6][3];      /* Position Trajectory - Present Position, rad, servo frame */
    float32_t profile_velocity[6][3];    /* Velocity Trajectory, rad/s, servo frame */
    uint16_t voltage_dv[6][3];  /* Present Input Voltage, 0.1 V */
    uint8_t temperature_c[6][3];         /* Present Temperature, degrees C */
    uint16_t min_voltage_dv;    /* lowest input voltage of the servos that answered; of the last read that had data when none did */
    bool valid[6][3];           /* this servo returned data; otherwise the values above are untouched */
    bool link_error;            /* a timeout or bus error: the receiver needs a reset */
} servo_read_t;

/**
 * Sync Read of load, velocity, position and the profile's own velocity and
 * position of all servos, timed and counted.
 *
 * Servos that returned data (including those reporting a hardware alert, whose
 * data is valid) are decoded into *out; the others are left untouched, so the
 * caller keeps their previous values. The return value is DNM_OK when every
 * servo returned data, otherwise the first reason a servo did not.
 */
static dynamixel_result_t servo_bus_read_state(servo_read_t *out)
{
    static uint32_t alert_mask;     /* servos reporting an alert, to log changes only */
    static uint16_t last_min_voltage_dv;

    uint8_t data[18 * XL430_STATE_LEN];
    dynamixel_result_t status[18];

    const uint32_t start = DWT->CYCCNT;
    (void) dynamixel_get_bytes_multiple(dynamixel_servos, 18, XL430_CT_RAM_PRESENT_LOAD,
                                        XL430_STATE_LEN, data, status);
    const uint32_t duration_us = diag_elapsed_us(start);

    dynamixel_result_t outcome = DNM_OK;
    uint32_t alerts = 0;
    uint32_t new_alert_mask = 0;
    bool link_blamed = false;
    out->link_error = false;
    out->min_voltage_dv = 0;

    for (int s = 0; s < 18; s++) {
        const dynamixel_result_t st = status[s];
        const bool usable = st == DNM_OK || st == XL430_STATUS_ALERT;
        out->valid[s / 3][s % 3] = usable;

        if (usable) {
            const uint8_t *d = &data[s * XL430_STATE_LEN];
            out->load[s / 3][s % 3] = xl430_load_to_percent(dynamixel_decode_i16(&d[0]));
            out->velocity[s / 3][s % 3] = xl430_velocity_to_radps(dynamixel_decode_i32(&d[2]));
            out->position[s / 3][s % 3] = dynamixel_decode_u32(&d[6]);
            out->profile_velocity[s / 3][s % 3] = xl430_velocity_to_radps(dynamixel_decode_i32(&d[10]));
            out->profile_position[s / 3][s % 3] = dynamixel_decode_u32(&d[14]);
            const uint16_t volt = (uint16_t) ((uint16_t) d[18] | ((uint16_t) d[19] << 8));
            out->voltage_dv[s / 3][s % 3] = volt;
            out->temperature_c[s / 3][s % 3] = d[20];
            if (out->min_voltage_dv == 0 || volt < out->min_voltage_dv) {
                out->min_voltage_dv = volt;
            }
            out->tracking_error[s / 3][s % 3] =
                (float32_t) (dynamixel_decode_i32(&d[14]) - dynamixel_decode_i32(&d[6])) * XL430_PULSE_RAD;
            if (st == XL430_STATUS_ALERT) {
                alerts++;
                new_alert_mask |= 1UL << s;
            }
            continue;
        }

        if (outcome == DNM_OK) {
            outcome = st;
        }
        if (st == DNM_LL_TIMEOUT || st == DNM_LL_ERR) {
            // The servos behind a missing reply cannot answer either: blame only the first
            out->link_error = true;
            if (!link_blamed) {
                bus_diag_servo_failure((unsigned) s);
                link_blamed = true;
            }
        } else {
            bus_diag_servo_failure((unsigned) s);
        }
    }

    // A missing reply from the first servo leaves every servo silent: report the last known voltage then
    if (out->min_voltage_dv == 0) {
        out->min_voltage_dv = last_min_voltage_dv;
    } else {
        last_min_voltage_dv = out->min_voltage_dv;
    }

    if (alerts > 0) {
        bus_diag_event(BUS_EV_SERVO_ALERT, alerts);
    }
    if (new_alert_mask != alert_mask) {
        LOG_WARN("[ServoTask] servo hardware alert mask 0x%05lx (was 0x%05lx), servo index = leg*3+joint",
                 (unsigned long) new_alert_mask, (unsigned long) alert_mask);
        alert_mask = new_alert_mask;
    }

    bus_diag_record(BUS_OP_READ_STATE, outcome, duration_us);
    return outcome;
}

/** The fixed profile for all servos. */
static void servo_fixed_profiles(uint32_t acceleration[6][3], uint32_t velocity[6][3])
{
    const uint32_t a = xl430_radps2_to_profile_acceleration(SERVO_PROFILE_ACCELERATION);
    const uint32_t v = xl430_radps_to_profile_velocity(SERVO_PROFILE_VELOCITY);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            acceleration[i][j] = a;
            velocity[i][j] = v;
        }
    }
}

#if SERVO_PROFILE_MODE == SERVO_PROFILE_MODE_COMPUTED
/**
 * A profile per servo that covers the move to its new goal in about one
 * control period. The move is measured from where the servo's own profile is
 * (Position Trajectory, read this tick): it includes what is left of the
 * previous goal, and unlike the measured position it does not lag. A servo
 * that did not answer the read is measured from its previous goal.
 */
static void servo_computed_profiles(uint32_t goals[6][3], const servo_read_t *read,
                                    uint32_t previous_goals[6][3], const float32_t duration_s,
                                    uint32_t acceleration[6][3], uint32_t velocity[6][3])
{
    const float32_t time_s = SERVO_PROFILE_T_FACTOR * duration_s;
    const float32_t ramp_s = SERVO_PROFILE_TR_FRACTION * time_s;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            const uint32_t from = read->valid[i][j] ? read->profile_position[i][j] : previous_goals[i][j];
            const float32_t delta_rad = (float32_t) ((int32_t) goals[i][j] - (int32_t) from) * XL430_PULSE_RAD;
            servo_profile_compute(delta_rad, time_s, ramp_s, &velocity[i][j], &acceleration[i][j]);
        }
    }
}
#endif

/**
 * Sync Write of the profile and goal position of all servos, timed and
 * counted: Profile Acceleration, Profile Velocity and Goal Position, 12 bytes
 * per servo at address 108. Every goal carries its profile, so no value left
 * in the servos' RAM (they keep it over a board reset) is ever used.
 */
static dynamixel_result_t servo_bus_write_goals(uint32_t goals[6][3], uint32_t acceleration[6][3],
                                                uint32_t velocity[6][3])
{
    uint8_t data[18 * SERVO_PROFILE_BYTES];
    servo_profile_pack(&acceleration[0][0], &velocity[0][0], &goals[0][0], 18, data);

    const uint32_t start = DWT->CYCCNT;
    const dynamixel_result_t res = dynamixel_set_bytes_multiple(
        dynamixel_servos, 18, XL430_CT_RAM_PROFILE_ACC, SERVO_PROFILE_BYTES, data);
    bus_diag_record(BUS_OP_WRITE_GOAL, res, diag_elapsed_us(start));
    return res;
}

/** Joint targets (compensated frame) to goal positions in pulses. */
static void servo_targets_to_goals(float32_t targets[6][3], uint32_t goals[6][3])
{
    float32_t angles[3];
    for (int i = 0; i < 6; i++) {
        uncompensate(targets[i], angles);
        goals[i][0] = xl430_rad_centered_to_pulse(angles[0]);
        goals[i][1] = xl430_rad_centered_to_pulse(angles[1]);
        goals[i][2] = xl430_rad_centered_to_pulse(angles[2]);
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

    // Wait until the servo task reports ready
    osEventFlagsWait(systemEventsHandle,
                         EVT_CONTROLLER_READY,
                         osFlagsWaitAll,
                         osWaitForever);

    uint32_t tick_count = osKernelGetTickCount();

    // One timer per report, all started now: reports with the same period come out
    // together, in the order below.
    const uint32_t start_ms = (uint32_t) osKernelGetTickCount() * portTICK_PERIOD_MS;
    periodic_t status_timer, load_timer, stack_timer;
    periodic_init(&status_timer, STATUS_REPORT_PERIOD_S * 1000U, start_ms);
    periodic_init(&load_timer, LOAD_REPORT_PERIOD_S * 1000U, start_ms);
    periodic_init(&stack_timer, STACK_REPORT_PERIOD_S * 1000U, start_ms);

    /* Infinite loop */
    for (;;) {
        const uint32_t now_ms = (uint32_t) osKernelGetTickCount() * portTICK_PERIOD_MS;

        if (periodic_due(&status_timer, now_ms)) {
            report_bus();
            report_servo_health();
            report_sensors();
            report_accel_gate();
        }
        if (periodic_due(&load_timer, now_ms)) {
            report_load();
        }
        if (periodic_due(&stack_timer, now_ms)) {
            report_stacks();
        }

        // The reports above are timed by their own timers, this only sets how finely
        tick_count += MAIN_LOOP_INTERVAL;
        osDelayUntil(tick_count);
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

    // Frames of the command link (cmd_frame.h): every transfer is CMD_FRAME_LEN bytes
    // in both directions. The master clocks out a command and clocks in the status
    // prepared before the transfer, so the status lags the command by one frame.
    // Hardware NSS frames each transfer; a damaged or cut short transfer fails the
    // CRC and the link recovers at the next one.
    static uint8_t rx_frame[CMD_FRAME_LEN];
    static uint8_t tx_frame[CMD_FRAME_LEN];
    uint32_t last_warn_ms = 0;
    bool warned = false;

    /* Infinite loop */
    for (;;) {
        cmd_link_get_reply(tx_frame);

        if (HAL_SPI_TransmitReceive_IT(&hspi1, tx_frame, rx_frame, CMD_FRAME_LEN) != HAL_OK) {
            LOG_ERROR("[StartSpiSlaveTask] HAL_SPI_TransmitReceive error");

            // Abort and reset SPI peripheral state
            HAL_SPI_Abort(&hspi1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const uint32_t flags = osThreadFlagsWait(SPI1_RX_CPLT | SPI1_ERROR, osFlagsWaitAny, portMAX_DELAY);
        if ((flags & 0x80000000U) != 0U || (flags & SPI1_ERROR) != 0U) {
            // osThreadFlagsWait error or SPI error (overrun, mode fault): start over
            HAL_SPI_Abort(&hspi1);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        const uint32_t now_ms = osKernelGetTickCount() * portTICK_PERIOD_MS;
        if (cmd_link_on_frame(rx_frame, CMD_FRAME_LEN, now_ms) == CMD_RX_BAD) {
            // Possibly out of step with the master: drop whatever is pending
            HAL_SPI_Abort(&hspi1);

            // No logging per frame (25 Hz and up), at most one line every 5 s
            if (!warned || (uint32_t) (now_ms - last_warn_ms) >= 5000U) {
                cmd_rx_t stats;
                cmd_link_get_stats(&stats);
                LOG_WARN("[StartSpiSlaveTask] bad command frame (error %d), crc %lu, other %lu, ok %lu",
                         (int) stats.last_error, (unsigned long) stats.crc_errors,
                         (unsigned long) stats.rejected, (unsigned long) stats.accepted);
                last_warn_ms = now_ms;
                warned = true;
            }
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
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        // Only one task can use SPI at a time
        if (osMutexAcquire(spiMutexHandle, 1) != osOK) {
            continue;
        }

        // Read BMI088 gyro (SPI)
        if (bmi08g_get_data(&gyro, &bmi088) != 0) {
            LOG_ERROR("[GyroTask] Failed to get data");
            osMutexRelease(spiMutexHandle);   // without this every later read times out for good
            continue;
        }

        osMutexRelease(spiMutexHandle);

        // Scale and convert to rad/s
        sensor_frame_data[0] = stm32_bmi08g_scale_data(gyro.x, range) * (float32_t)(M_PI / 180.0f);
        sensor_frame_data[1] = stm32_bmi08g_scale_data(gyro.y, range) * (float32_t)(M_PI / 180.0f);
        sensor_frame_data[2] = stm32_bmi08g_scale_data(gyro.z, range) * (float32_t)(M_PI / 180.0f);
        stm32_bmi08g_sensor_to_ned(sensor_frame_data, sample.data);

        sample.type = SENSOR_GYRO;
        sample.tick = osKernelGetTickCount();

        // Non-blocking send (gyro is high rate)
        osMessageQueuePut(ekf_queueHandle, &sample, 0, 0);
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


    accel_gate_init(&accel_gate, ACCEL_GATE_DEFAULT_TOLERANCE);
    accel_interrupt_enable = 1;

    uint8_t range = bmi088.accel_cfg.range;
    float32_t sensor_frame_data[3];

    /* Infinite loop */
    for (;;) {
        // Wait for interrupt
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        // Only one task can use SPI at a time
        if (osMutexAcquire(spiMutexHandle, 5) != osOK) {
            continue;
        }

        // Read BMI088 gyro (SPI)
        if (bmi08a_get_data(&accel, &bmi088) != 0) {
            LOG_ERROR("[AccelTask] Failed to get data");
            osMutexRelease(spiMutexHandle);   // without this every later read times out for good
            continue;
        }

        osMutexRelease(spiMutexHandle);

        // Scale and convert to m/s2
        sensor_frame_data[0] = stm32_bmi08a_scale_data(accel.x, range) * GRAVITY / 1000;
        sensor_frame_data[1] = stm32_bmi08a_scale_data(accel.y, range) * GRAVITY / 1000;
        sensor_frame_data[2] = stm32_bmi08a_scale_data(accel.z, range) * GRAVITY / 1000;
        stm32_bmi08a_sensor_to_ned(sensor_frame_data, sample.data);

        // Roll and pitch come from the direction of gravity: a sample that is not 1 g long is not gravity
        // (a corrupted read, an impact) and would pull the attitude off. The gyro carries the estimate meanwhile.
        const int16_t accel_raw[3] = {accel.x, accel.y, accel.z};
        if (!accel_gate_check(&accel_gate, sample.data, accel_raw)) {
            continue;
        }

        sample.type = SENSOR_ACCEL;
        sample.tick = osKernelGetTickCount();

        // Blocking send (accel is not that high rate)
        osMessageQueuePut(ekf_queueHandle, &sample, 0, 5);
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
    osEventFlagsWait(systemEventsHandle,
                EVT_EKF_READY,
                osFlagsWaitAll,
                osWaitForever);

    sensor_sample_t sample;
    struct bmm350_mag_temp_data data;
    float32_t sensor_frame_data[3];

    uint32_t tick_count = osKernelGetTickCount();

    // float32_t bias[3] = { +37, -24, +35 };
    float32_t bias[3] = { 0, 0, 0 };

    /* Infinite loop */
    for(;;) {
        if (bmm350_get_compensated_mag_xyz_temp_data(&data, &bmm350) < 0) {
            // LOG_ERROR("[MagTask] Failed to get data");
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
            osMessageQueuePut(ekf_queueHandle, &sample, 0, 10);
        }

        tick_count += MAG_LOOP_INTERVAL;
        osDelayUntil(tick_count);
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
    uint32_t last_gyro_tick = 0;

    attitude_ekf_init(&ekf);

    // Wait for the controller to become ready
    osEventFlagsWait(systemEventsHandle,
                EVT_CONTROLLER_READY,
                osFlagsWaitAll,
                osWaitForever);

    // Signal ready
    osEventFlagsSet(systemEventsHandle, EVT_EKF_READY);

    for (;;) {
        if (osMessageQueueGet(ekf_queueHandle, &sample, NULL, osWaitForever) == osOK) {

            if (sample.type == SENSOR_GYRO) {
                float32_t dt = (float32_t)(sample.tick - last_gyro_tick) * portTICK_PERIOD_MS * 0.001f;
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

/* USER CODE BEGIN Header_StartUsartRxTask */
/**
* @brief Function implementing the usartRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartRxTask */
void StartUsartRxTask(void *argument)
{
  /* USER CODE BEGIN StartUsartRxTask */
    assert(argument != NULL);
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)argument;

    // Enabl the IDLE interrupt, not standard in the HAL
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    HAL_HalfDuplex_EnableReceiver(huart);
    if (HAL_UART_Receive_DMA(huart, dma_rx_buf, DMA_RX_BUF_SIZE) != HAL_OK) {
        LOG_ERROR("[USARTRX] Failed to start receiver");
        Error_Handler();
    };

    osEventFlagsSet(systemEventsHandle, EVT_UART_READY);

  /* Infinite loop */
    for(;;)
    {
        const uint32_t flags = osThreadFlagsWait(RX_DMA_HT | RX_DMA_TC | RX_DMA_IDLE | RX_DMA_ERROR, osFlagsWaitAny,
                                           osWaitForever);

        if (flags & (RX_DMA_HT | RX_DMA_TC | RX_DMA_IDLE)) {
            uart_rx_drain_dma();
        }

        if (flags & RX_DMA_ERROR) {
            /* HAL aborts the DMA on framing/overrun/noise errors; if we don't
             * rearm here the receiver stays wedged until the next
             * uart_rx_reset_dma() from the servo task. Any partially-received
             * bytes are unreliable, so flush them. */
            LOG_WARN("[USARTRX] UART error, resetting DMA receiver");
            bus_diag_event(BUS_EV_RESET_UART_ERR, 1);
            uart_rx_reset_dma();
        }
    }
  /* USER CODE END StartUsartRxTask */
}

/* USER CODE BEGIN Header_StartUsartTxTask */
/**
* @brief Function implementing the usartTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTxTask */
void StartUsartTxTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTxTask */
    assert(argument != NULL);
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)argument;

    tx_msg_t msg;
  /* Infinite loop */
    for(;;)
    {
        osMessageQueueGet(usart_tx_queueHandle, &msg, NULL, osWaitForever);

        // Acquire DMA mutex before stopping/starting DMA
        osMutexAcquire(uart_dma_mutexHandle, osWaitForever);

        // Stop DMA receive
        if (HAL_UART_DMAStop(huart) != HAL_OK) {
            LOG_ERROR("[USARTTX] Failed to stop receiver");
        };

        // Reset for next interation
        taskENTER_CRITICAL();
        dma_rx_read_idx = 0;
        huart->hdmarx->Instance->NDTR = DMA_RX_BUF_SIZE;
        taskEXIT_CRITICAL();

        // Enable Transmit
        if (HAL_HalfDuplex_EnableTransmitter(huart) != HAL_OK) {
            LOG_ERROR("[USARTTX] Failed to enable transmitter");
        };

        // Just to be sure
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);

        // Start the transfer. HAL_UART_TxCpltCallback fires from the TC IRQ
        // when the line is physically idle and rearms the receiver before
        // setting TX_DMA_TC, so by the time we wake up RX DMA is already
        // running.
        if (HAL_UART_Transmit_DMA(huart, msg.data, msg.len) != HAL_OK) {
            LOG_ERROR("[USARTTX] Failed to start transfer");
        };

        osThreadFlagsWait(TX_DMA_TC, osFlagsWaitAny, osWaitForever);

        // Release DMA mutex after restart complete
        osMutexRelease(uart_dma_mutexHandle);

        // Signal the write that transfer is complete
        osSemaphoreRelease(usart_tx_semHandle);
    }
  /* USER CODE END StartUsartTxTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
    UNUSED(argument);

    // Wait until the servo task reports ready
    osEventFlagsWait(systemEventsHandle,
                         EVT_SERVO_READY,
                         osFlagsWaitAll,
                         osWaitForever);

    uint32_t tick_count = osKernelGetTickCount();

    controller_ctx_t controller_ctx;
    float32_t actual_joint_angles[6][3] = {{0.0f}};
    float32_t target_joint_angles[6][3] = {{0.0f}};
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

    // The SPI link carries a twist, the controller still takes a speed along its
    // own heading and an absolute heading: the adapter bridges the two (T7.5).
    const adapter_cfg_t adapter_cfg = {
        .max_speed = 100.0f,
        .max_yaw_rate = CTRL_MAX_YAW_RATE,
        .min_height = 50.0f,
        .max_height = 170.0f,
        .max_accel = 200.0f,
        .link_timeout_ms = 300,
    };
    adapter_t adapter;
    adapter_report_t link_report = {0};
    adapter_init(&adapter, &adapter_cfg, 0.0f, CTRL_BODY_Z);

    osEventFlagsSet(systemEventsHandle, EVT_CONTROLLER_READY);

    TickType_t last_ticks = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
      TickType_t now = osKernelGetTickCount();
      TickType_t dt_ticks = now - last_ticks;
      last_ticks = now;

      float dt_s = (float)dt_ticks * (float)portTICK_PERIOD_MS * 1e-3f;
      // Bounded around the 100ms CONTROL_LOOP_INTERVAL: half-tick to double-tick.
      // Revisit if the controlTask rate changes.
      dt_s = clampf(dt_s, 0.05f, 0.2f);

      // Update control information from the command link
      cmd_frame_t link_cmd;
      uint32_t link_rx_ms;
      const uint32_t now_ms = (uint32_t) now * portTICK_PERIOD_MS;
      if (cmd_link_take(&link_cmd, &link_rx_ms)) {
          adapter_set_command(&adapter, &link_cmd, link_rx_ms);
      }
      adapter_output_t adapted;
      adapter_step(&adapter, now_ms, dt_s, &adapted, &link_report);
      cmd.velocity = adapted.velocity;
      cmd.heading = adapted.heading;
      cmd.height = adapted.height;

      // Update the actual from the shared state
      servo_copy_actual_joint_angles(actual_joint_angles, &servo_shared_state);
      for (int i = 0; i < 6; i++) {
          struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];

          current_leg_state->actual_joint_angles[0] = actual_joint_angles[i][0];
          current_leg_state->actual_joint_angles[1] = actual_joint_angles[i][1];
          current_leg_state->actual_joint_angles[2] = actual_joint_angles[i][2];
      }

      controller_update(&controller_ctx, &attitude, &cmd, dt_s);

      // Status for the replies on the command link. The power down request and fault
      // clear flags are not acted on yet (no controller support), which is reported
      // as limited. The contact is not measured yet (T7.6): the flag is not set.
      const status_frame_t link_status = {
          .seq = link_report.seq,
          .state = (uint8_t) controller_ctx.state,
          .fault = 0,
          .flags = (uint8_t) ((link_report.link_ok ? STATUS_FLAG_LINK_OK : 0U) |
                              ((link_report.limited || link_report.powerdown_request || link_report.fault_clear)
                                   ? STATUS_FLAG_LIMITED
                                   : 0U)),
          .contact = 0,
          .yaw = wrap_to_pi(controller_ctx.robot.hexapod.rotation[2]),
      };
      cmd_link_publish_status(&link_status);

      // Update the shared state from the controller state
      for (int i = 0; i < 6; i++) {
          struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];

          target_joint_angles[i][0] = current_leg_state->next_joint_angles[0];
          target_joint_angles[i][1] = current_leg_state->next_joint_angles[1];
          target_joint_angles[i][2] = current_leg_state->next_joint_angles[2];
      }
      servo_set_target_joint_angles(&servo_shared_state, target_joint_angles, dt_s);

      tick_count += CONTROL_LOOP_INTERVAL;
      osDelayUntil(tick_count);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
    UNUSED(argument);
    servo_state_t state = SERVO_INIT;

    uint32_t values[6][3] = {0};
    uint32_t written_goals[6][3] = {0};     // the goals of the last write
    uint32_t profile_acceleration[6][3] = {0};
    uint32_t profile_velocity[6][3] = {0};
    float32_t joint_angles[6][3] = {{ 0.0f }};
    float32_t target_joint_angles[6][3] = {{0.0f}};
    float32_t target_duration_s = 0.0f;
    uint32_t target_seq = 0;
    uint32_t written_seq = 0;               // the targets last written to the servos
    bool goals_written = false;             // that write succeeded

    int limit_alert = 0;
    bool request_powerdown = false;
    bool limit_alert_enabled = false;

    // Last values read from each servo, kept so that a servo that does not
    // answer in one cycle keeps its previous value
    servo_read_t servo_read = {0};
    float32_t servo_angles_rad[6][3] = {{0.0f}};   // servo frame, before compensate()

    osEventFlagsWait(systemEventsHandle, EVT_UART_READY, osFlagsWaitAll, osWaitForever);

    uint32_t tick_count = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
      limit_alert = 0;

      servo_get_flags(&servo_shared_state, &request_powerdown, &limit_alert_enabled);
      if (request_powerdown && state != SERVO_IDLE) {
          state = SERVO_POWER_DOWN;
      }
      if (!request_powerdown && state == SERVO_IDLE) {
          state = SERVO_POWER_UP;
      }

    switch (state) {
        case SERVO_INIT:
            PERIF_Dynamixel_Init();
            PERIF_Dynamixel_Configure();
            state = SERVO_POWER_UP;
            break;
        case SERVO_SYNC_FROM_HW: {
            // We use the fact that the dynamixel servo list is ordered
            // similar to our [6][3] structures but flattened

            // A) Read the actual positions from the servos
            //    - Convert pulses to rad
            //    - Compensate for geometry
            //    We need every servo here: the targets start from what was read
            const dynamixel_result_t res = servo_bus_read_state(&servo_read);
            if (res != DNM_OK) {
                LOG_ERROR("[ServoTask] Failed to read actual servo positions [%d]", res);
                break;
            };

            for (int i=0; i<6; i++) {
                for (int j=0; j<3; j++) {
                    servo_angles_rad[i][j] = xl430_pulse_to_rad_centered(servo_read.position[i][j]);
                }
                compensate(servo_angles_rad[i], joint_angles[i]);
            }

            // B) Update the shared state to the current state of the hardware
            servo_set_actual_and_target_joint_angles(&servo_shared_state, joint_angles);
            servo_set_load_and_velocity(&servo_shared_state, servo_read.load, servo_read.velocity);
            servo_set_health(&servo_shared_state, servo_read.voltage_dv, servo_read.temperature_c);

            // C) Proceed to the next state
            state = SERVO_SYNC_TO_HW;
            break;
        }
        case SERVO_SYNC_TO_HW:
            // A) Convert the current targets to pulses
            servo_copy_target_command(target_joint_angles, &target_duration_s, &target_seq, &servo_shared_state);
            servo_targets_to_goals(target_joint_angles, values);

            // B) Write the goal position and its profile to the servos
            //    The fixed profile: there is no previous move to follow on
            //    A failed write is repeated in SERVO_RUNNING
            servo_fixed_profiles(profile_acceleration, profile_velocity);
            goals_written = servo_bus_write_goals(values, profile_acceleration, profile_velocity) == DNM_OK;
            memcpy(written_goals, values, sizeof(written_goals));
            written_seq = target_seq;
            if (!goals_written) {
                LOG_ERROR("[ServoTask] Failed to write target servo positions");
            };

            // C) Signal ready and proceed to the next state
            osEventFlagsSet(systemEventsHandle, EVT_SERVO_READY);
            state = SERVO_RUNNING;
            break;
        case SERVO_RUNNING: {
            // Snapshot controller targets to avoid torn reads during updates
            servo_copy_target_command(target_joint_angles, &target_duration_s, &target_seq, &servo_shared_state);

            // A) Read the actual positions from the servos
            //    - Convert pulses to rad
            //    - Compensate for geometry
            //    A servo that did not answer keeps its last known position
            const dynamixel_result_t res = servo_bus_read_state(&servo_read);
            for (int i=0; i<6; i++) {
                for (int j=0; j<3; j++) {
                    if (servo_read.valid[i][j]) {
                        servo_angles_rad[i][j] = xl430_pulse_to_rad_centered(servo_read.position[i][j]);
                    }
                }
                compensate(servo_angles_rad[i], joint_angles[i]);
            }
            servo_set_load_and_velocity(&servo_shared_state, servo_read.load, servo_read.velocity);
            servo_set_profile_tracking(&servo_shared_state, servo_read.tracking_error, servo_read.profile_velocity);
            servo_set_health(&servo_shared_state, servo_read.voltage_dv, servo_read.temperature_c);

            if (res != DNM_OK) {
                LOG_WARN("[ServoTask] Failed to read all servos (error %d)%s, lowest input %u.%u V", res,
                         servo_read.link_error ? ", resetting UART" : "",
                         (unsigned) (servo_read.min_voltage_dv / 10U), (unsigned) (servo_read.min_voltage_dv % 10U));

                if (servo_read.link_error) {
                    // Wait 5ms for any pending servo responses from the BULK_READ/SYNC_READ
                    // to arrive on the bus (18 servos may still be transmitting)
                    osDelay(5);

                    // Perform DMA recovery (mutex-protected)
                    bus_diag_event(BUS_EV_RESET_READ_FAIL, 1);
                    uart_rx_reset_dma();
                }
                // Damaged replies were read in full, the receiver is still in step
            };

            // B) Write new targets, once per set the controller publishes
            //    - Limits check
            //    - Uncompensate angles, convert to pulses
            //    - A profile per joint (or the fixed one), see SERVO_PROFILE_MODE
            //    - Write the goals with their profile; the servos interpolate
            //    A failed write is repeated on the next tick
            if (target_seq != written_seq || !goals_written) {
                for (int i=0; i<6; i++) {
                    for (int j=0; j<3; j++) {
                        if (limit_alert_enabled) {
                            if (target_joint_angles[i][j] < r.leg[i].limits[j][0] || target_joint_angles[i][j] > r.leg[i].limits[j][1]) {
                                LOG_ERROR("Limit alert triggered, leg %d, axis %d", i, j);
                                LOG_ERROR("Calculated value %5.2f, limits %5.2f, %5.2f", target_joint_angles[i][j], r.leg[i].limits[j][0], r.leg[i].limits[j][1]);
                                limit_alert = 1;
                            }
                        }
                    }
                }

                if (limit_alert && limit_alert_enabled) {
                    // TODO instead of error send a signal to the controller to recover
                    state = SERVO_ERROR;
                    break;
                }

                servo_targets_to_goals(target_joint_angles, values);
#if SERVO_PROFILE_MODE == SERVO_PROFILE_MODE_COMPUTED
                servo_computed_profiles(values, &servo_read, written_goals, target_duration_s,
                                        profile_acceleration, profile_velocity);
#else
                servo_fixed_profiles(profile_acceleration, profile_velocity);
#endif
                goals_written = servo_bus_write_goals(values, profile_acceleration, profile_velocity) == DNM_OK;
                memcpy(written_goals, values, sizeof(written_goals));
                written_seq = target_seq;
                if (!goals_written) {
                    LOG_ERROR("[ServoTask] Failed to write target servo positions");
                };
            }

            // C) Write the actual readings into the shared state
            servo_set_actual_joint_angles(&servo_shared_state, joint_angles);

            break;
        }
        case SERVO_ERROR:
            // disable torque on all motors
            for (int i=0; i<18; i++) {
                dynamixel_set_torque_enable(&dynamixel_servos[i], 0);
                dynamixel_set_led(&dynamixel_servos[i], 0);
            }
            Error_Handler();
            // notify error
            break;
        case SERVO_POWER_DOWN:
            LOG_INFO("[ServoTask] entering POWER_DOWN");
            // disable torque on all motors
            for (int i=0; i<18; i++) {
                dynamixel_result_t r = dynamixel_set_torque_enable(&dynamixel_servos[i], 0);
                if (r != DNM_OK) {
                    LOG_WARN("[ServoTask] torque disable failed for servo %d (err %d)",
                             dynamixel_servos[i].id, r);
                }
                dynamixel_set_led(&dynamixel_servos[i], 0);
            }
            state = SERVO_IDLE;
            break;
        case SERVO_IDLE:
            // Nothing to do here
            break;
        case SERVO_POWER_UP:
            for (int i=0; i<18; i++) {
                dynamixel_set_torque_enable(&dynamixel_servos[i], 1);
                dynamixel_set_led(&dynamixel_servos[i], 1);
            }
            state = SERVO_SYNC_FROM_HW;
            break;
    }

    tick_count += pdMS_TO_TICKS(SERVO_LOOP_INTERVAL);
    if ((int32_t)(osKernelGetTickCount() - tick_count) > 0) {
        // The iteration took longer than its period
        bus_diag_event(BUS_EV_LOOP_OVERRUN, 1);
    }
    osDelayUntil(tick_count);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_rgb_led_task */
/**
* @brief Function implementing the rgbLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rgb_led_task */
__weak void rgb_led_task(void *argument)
{
  /* USER CODE BEGIN rgb_led_task */
    UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rgb_led_task */
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
    /* HSE did not start: carry on from the HSI instead of halting */
    if (clock_hse_fallback()) {
        return;
    }

    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    log_sink_uart_panic();   // queued log output has to go out by polling from here

    /* Set indicator LED to red (active low: only the red pin is driven low) */
    HAL_GPIO_WritePin(ST_LED_R_GPIO_Port, ST_LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_LED_G_GPIO_Port, ST_LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST_LED_B_GPIO_Port, ST_LED_B_Pin, GPIO_PIN_SET);

    printf("\r\n\r\n=== FATAL ERROR ===\r\n");
    osThreadId_t currentThread = osThreadGetId();
    if (currentThread != NULL) {
        printf("Thread: %s\r\n", osThreadGetName(osThreadGetId()));
    } else {
        printf("Threads not started yet\r\n");
    }

    error_print_backtrace();

    printf("System halted.\r\n");

    for (;;) {
        __BKPT(0);   // Optional: break into debugger
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
