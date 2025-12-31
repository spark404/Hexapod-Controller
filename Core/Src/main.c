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

#include "bmm350.h"
#include "bmi08x.h"
#include "bno055.h"

#include "dynamixel/dynamixel.h"
#include "dynamixel_ll_uart.h"

#include "hexapodmath/additional_functions.h"


#include "robot.h"
#include "robot_config.h"
#include "calculator.h"
#include "servos.h"
#include "log.h"
#include "semphr.h"
#include "dynamixel/protocol.h"
#include "controller.h"
#include "rgb_led.h"

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
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 64,
    .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t spiSlaveTaskHandle;
const osThreadAttr_t spiSlaveTask_attributes = {
    .name = "spiSlaveTask",
    .stack_size = 128 * 16,
    .priority = (osPriority_t) osPriorityNormal,
};

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

uint8_t led_state[3] = {1, 0, 0};

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

void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartSpiSlaveTask(void *argument);

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int g_log_level = LOG_LEVEL_DEBUG;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim1);

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
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

    /* USER CODE BEGIN RTOS_THREADS */
    spiSlaveTaskHandle = osThreadNew(StartSpiSlaveTask, NULL, &spiSlaveTask_attributes);
    rgb_led_init();
    /* USER CODE END RTOS_THREADS */

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
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
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
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_I2C2_Init(void) {
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
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
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
static void MX_I2C3_Init(void) {
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
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
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
static void MX_SPI1_Init(void) {
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
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
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
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 16 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
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
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void) {
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
    if (HAL_HalfDuplex_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART6_Init 2 */

    /* USER CODE END USART6_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, SPI2_CS_ACC_Pin | SPI2_CS_GYR_Pin | ST_LED_G_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, ST_LED_B_Pin | ST_LED_R_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : SPI2_CS_ACC_Pin SPI2_CS_GYR_Pin ST_LED_G_Pin */
    GPIO_InitStruct.Pin = SPI2_CS_ACC_Pin | SPI2_CS_GYR_Pin | ST_LED_G_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : SPI2_INT_ACC_Pin SPI2_INT_GYR_Pin */
    GPIO_InitStruct.Pin = SPI2_INT_ACC_Pin | SPI2_INT_GYR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : ST_LED_B_Pin ST_LED_R_Pin */
    GPIO_InitStruct.Pin = ST_LED_B_Pin | ST_LED_R_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartSpiSlaveTask(void *argument) {
    (void) argument;

    // Message format (7 bytes)
    //   uint8_t magic
    //   uint8_t reserved
    //   uint8_t register
    //   uint32_t value
    uint8_t buffer[16];

    for (;;) {
        LOG_INFO("[StartSpiSlaveTask] Waiting for receive...");

        // Start length byte reception
        if (HAL_SPI_Receive_IT(&hspi1, buffer, 7) != HAL_OK) {
            LOG_ERROR("[StartSpiSlaveTask] HAL_SPI_Receive error");

            // Wait, reset and try again
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const uint32_t flags = osThreadFlagsWait(SPI_RX_CPLT | SPI_ERR, osFlagsWaitAny, portMAX_DELAY);
        if (flags == (uint32_t) osErrorTimeout) {
            LOG_DEBUG("[StartSpiSlaveTask] osThreadFlagsWait timeout");
            continue;
        }

        if (flags & (1U << 31)) {
            LOG_DEBUG("[StartSpiSlaveTask] osThreadFlagsWait error %d", flags);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (flags == SPI_ERR) {
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
            case 0x01:
                // Command set speed
                const float32_t *new_velocity = (float32_t *)&buffer[3];
                if (*new_velocity < 0 || *new_velocity > 100) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new velocity %5.2f", *new_velocity);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set speed to %5.2f mm/s", *new_velocity);
                updated_velocity = *new_velocity;
                break;
            case 0x02:
                // Command set heading
                const float32_t *new_heading = (float32_t *)&buffer[3];
                if (*new_heading < 0 || *new_heading > M_PI) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new heading %5.3f rad", *new_heading);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set heading to %5.3f rad", *new_heading);
                updated_heading = *new_heading;
                break;
            case 0x03:
                // Command set height
                const float32_t *new_height = (float32_t *)&buffer[3];
                if (*new_height < 50 || *new_height > 170) {
                    LOG_WARN("[StartSpiSlaveTask] Ignoring new body height %5.2f", *new_height);
                    break;
                }
                LOG_INFO("[StartSpiSlaveTask] Set new body height to %5.2f mm", *new_height);
                updated_height = *new_height;
                break;
            default:
                LOG_WARN("[StartSpiSlaveTask] Unknown command: 0x%02x", buffer[2]);
                break;
        }
    }
}

int __io_putchar(int ch) {
    while (!LL_USART_IsActiveFlag_TXE(USART1)) {
    }

    LL_USART_TransmitData8(USART1, ch);

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

    // Give semaphore to unblock the receive task
    osThreadFlagsSet(spiSlaveTaskHandle, SPI_RX_CPLT);

    // Request context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    LOG_DEBUG("HAL_SPI_ErrorCallback: %ld", hspi->ErrorCode);

    // Give semaphore to unblock the receive task
    osThreadFlagsSet(spiSlaveTaskHandle, SPI_ERR);

    // Request context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void stm32_state_change_cb(
    controller_ctx_t *ctx,
    controller_state_t from,
    controller_state_t to,
    void *user_data
) {
    (void)ctx;
    (void)user_data;

    switch (to) {
        case CTRL_SYNCING:
            rgb_led_set_color(RGB_LED_COLOR_RED);
            rgb_led_blink(500, 250);
            for (int i = 0; i < 3 * 6; i++) {
                LOG_INFO("Activating servo %d...", i);
                dynamixel_set_torque_enable(&dynamixel_servo[i], 1);
                dynamixel_set_led(&dynamixel_servo[i], 1);
            }
            break;
        case CTRL_POWERDOWN:
            rgb_led_set_color(RGB_LED_COLOR_MAGENTA);
            rgb_led_blink(500, 250);
            for (int i = 0; i < 3 * 6; i++) {
                LOG_INFO("Deactivating servo %d...", i);
                dynamixel_set_torque_enable(&dynamixel_servo[i], 0);
                dynamixel_set_led(&dynamixel_servo[i], 0);
            }
            break;
        case CTRL_STANDUP:
            rgb_led_set_color(RGB_LED_COLOR_CYAN);
            rgb_led_blink(500, 250);
            break;
        case CTRL_STANDING:
            rgb_led_set_color(RGB_LED_COLOR_GREEN);
            rgb_led_blink(500, 250);
            break;
        case CTRL_WALKING:
            rgb_led_set_color(RGB_LED_COLOR_BLUE);
            rgb_led_blink(500, 250);
            break;
        default:
            break;
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
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN 5 */
    (void) argument;

    LOG_INFO("Hexapod Control Firmware");
    LOG_INFO("Build: %s %s", __DATE__, __TIME__);

    rgb_led_set_color(RGB_LED_COLOR_YELLOW);
    rgb_led_on();

    LOG_INFO("Starting device checks");

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

    int8_t bmi088_res = bmi08g_init(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 gyro initialization failed: %d", bmi088_res);
        // Error_Handler();
    } else {
        LOG_INFO("BMI088 gyro initialization complete!");
    }

    bmi088_res = bmi08a_init(&bmi088);
    if (bmi088_res != 0) {
        LOG_ERROR("BMI088 acc initialization failed: %d", bmi088_res);
        // Error_Handler();
    } else {
        LOG_INFO("BMI088 acc initialization complete!");
    }

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

#ifdef BNO055_PRESENT
    /* Setup BNO055 (on qwiic port) */
    bno055.bus_read = &stm32_bno055_bus_read;
    bno055.bus_write = &stm32_bno055_bus_write;
    bno055.delay_msec = &stm32_bno055_delay_us;
    bno055.dev_addr = 0x28;
    s8 bno055_res = bno055_init(&bno055);
    if (bno055_res != 0) {
        LOG_ERROR("BNO055 initialization failed: %d", bno055_res);
    } else {
        LOG_INFO("BNO055 initialization complete!");
    }
#endif

    rgb_led_set_color(RGB_LED_COLOR_CYAN);

    dynamixel_uart_context.huart = &huart6;
    dynamixel_uart_context.callerThread = osThreadGetId();

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

    rgb_led_set_color(RGB_LED_COLOR_GREEN);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    controller_ctx_t controller_ctx;
    controller_command_t cmd = {
        .velocity = 0,
        .heading = 0,
        .height = 100,
    };
    controller_init(&controller_ctx);
    controller_set_state_callback(&controller_ctx, stm32_state_change_cb, NULL);

    /* Infinite loop */
    for (;;) {
        cmd.velocity = updated_velocity;
        cmd.heading = updated_heading;
        cmd.height = updated_height;

        // Determine the actual servo positions
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];
            const struct leg *current_leg = &r.leg[i];

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

        controller_update(&controller_ctx, &cmd, MAIN_LOOP_INTERVAL / 1000);

        // Write next values to the servos
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &controller_ctx.robot.leg_state[i];
            const struct leg *current_leg = &r.leg[i];

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

        // Schedule at fixed 1 Hz
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_LOOP_INTERVAL));
    }
    /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
