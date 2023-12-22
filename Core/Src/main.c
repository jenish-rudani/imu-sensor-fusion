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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "lsm6dso_reg.h"
#include "lsm6dso_imu.h"
#include "complementary_filter.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUCLEO_U575Z
#define SENSOR_BUS hi2c1
//#define DEBUG_PRINT_XL
//#define DEBUG_PRINT_GY
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    LSM6DSO_BOOT_TIME      10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

SemaphoreHandle_t i2cMutex;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static TaskHandle_t xImuReadAndComputeH = NULL, xGetInclinationH = NULL;

static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
struct imu_data *imu_data;
struct cf_data *sf_cf_data;

rc_filter lpf_xl_coeff[3];
rc_filter lpf_gy_coeff[3];

stmdev_ctx_t dev_ctx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config (void);
static void SystemPower_Config (void);
static void MX_GPIO_Init (void);
static void MX_GPDMA1_Init (void);
static void MX_ICACHE_Init (void);
static void MX_USART1_UART_Init (void);
static void MX_I2C1_Init (void);

/* USER CODE BEGIN PFP */
static int32_t platform_write (void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read (void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com (uint8_t *tx_buffer, uint16_t len);
static void platform_delay (uint32_t ms);
static void v_imu_read_and_compute_task (void *pvParameters);
static void print_sf_cf_results (struct cf_data*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef DEBUG_PRINT
uint8_t Buffer[25] = { 0 };
uint8_t Space[] = "";
uint8_t StartMSG[] = "Starting I2C Scanning:\r\n";
uint8_t EndMSG[] = "Done!\r\n\r\n";
#endif

static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/**
 * @brief Remap printf to USART
 * @param None
 * @retval pointer to character buffer
 */
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
  return ch;
}

/**
 * @brief LSM6DSO Sensor Initialization Function
 * @param None
 * @retval None
 */
void lsm6dso_init () {

  do {
    /* Check device ID */
    lsm6dso_device_id_get(&dev_ctx, &whoamI);
    platform_delay(100);
  } while (whoamI != LSM6DSO_ID);

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_250dps);
  /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  lsm6dso_fifo_watermark_set(&dev_ctx, 100);
  /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, LSM6DSO_XL_BATCHED_AT_833Hz);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, LSM6DSO_GY_BATCHED_AT_833Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_STREAM_MODE);
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_1667Hz);
  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_1667Hz);
}

/**
 * @brief Read IMU data from Fifo and compute pitch, roll angle estimates
 * @param Takes Task Parameters
 * @retval None
 */
void v_imu_read_and_compute_task (void *pvParameters) {
  imu_data = (struct imu_data*) malloc(sizeof(struct imu_data));
  sf_cf_data = (struct cf_data*) (malloc(sizeof(struct cf_data)));
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

      uint16_t num = 0;
      uint16_t XLnum = 0;
      uint16_t GYnum = 0;
      bool xl_flag = false;
      bool gy_flag = false;
      uint8_t wm_flag = 0;
      lsm6dso_fifo_tag_t reg_tag;
      axis3bit16_t dummy;
      lsm6dso_fifo_wtm_flag_get(&dev_ctx, &wm_flag); /* Read fifo watermark flag */
      if (wm_flag > 0) {
        memset(sf_cf_data, 0x00, sizeof(sf_cf_data));
        lsm6dso_fifo_data_level_get(&dev_ctx, &num); /* Read number of samples in FIFO */
        XLnum = 0;
        GYnum = 0;
        while (num--) {
          /* Read FIFO tag */
          lsm6dso_fifo_sensor_tag_get(&dev_ctx, &reg_tag);

          switch (reg_tag) {
            case LSM6DSO_XL_NC_TAG:
              memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
              lsm6dso_fifo_out_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
              data_raw_acceleration.i16bit[0] = rc_filter_update(&lpf_xl_coeff[0], (float)data_raw_acceleration.i16bit[0]);
              data_raw_acceleration.i16bit[1] = rc_filter_update(&lpf_xl_coeff[1], (float)data_raw_acceleration.i16bit[1]);
              data_raw_acceleration.i16bit[2] = rc_filter_update(&lpf_xl_coeff[2], (float)data_raw_acceleration.i16bit[2]);
              imu_data->acc_x = (float_t) lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]) * G_MPS / 1000 - 0.01;
              imu_data->acc_y = (float_t) lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]) * G_MPS / 1000 + 0.01;
              imu_data->acc_z = (float_t) lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]) * G_MPS / 1000 + 0.02;
              imu_data->acc_rms = sqrt(imu_data->acc_x * imu_data->acc_x + imu_data->acc_y * imu_data->acc_y + imu_data->acc_z * imu_data->acc_z);
              xl_flag = true;
              XLnum++;
#ifdef DEBUG_PRINT_XL
            printf("XL:%4.2f\t%4.2f\t%4.2f\r\n",data_raw_acceleration.i16bit[0],data_raw_acceleration.i16bit[1],data_raw_acceleration.i16bit[2]);
#endif
              break;

            case LSM6DSO_GYRO_NC_TAG:
              memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
              lsm6dso_fifo_out_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
              data_raw_angular_rate.i16bit[0] = rc_filter_update(&lpf_gy_coeff[0], (float)data_raw_angular_rate.i16bit[0]);
              data_raw_angular_rate.i16bit[1] = rc_filter_update(&lpf_gy_coeff[1], (float)data_raw_angular_rate.i16bit[1]);
              data_raw_angular_rate.i16bit[2] = rc_filter_update(&lpf_gy_coeff[2], (float)data_raw_angular_rate.i16bit[2]);
              imu_data->gyr_x = (float_t) lsm6dso_from_fs250_to_mdps(data_raw_angular_rate.i16bit[0]) * DEG2RAD / 1000;
              imu_data->gyr_y = (float_t) lsm6dso_from_fs250_to_mdps(data_raw_angular_rate.i16bit[1]) * DEG2RAD / 1000;
              imu_data->gyr_z = (float_t) lsm6dso_from_fs250_to_mdps(data_raw_angular_rate.i16bit[2]) * DEG2RAD / 1000;
              gy_flag = true;
              GYnum++;
#ifdef DEBUG_PRINT_GY
            printf("GY:%4.2f\t%4.2f\t%4.2f\r\n",data_raw_angular_rate.i16bit[0],data_raw_angular_rate.i16bit[0],data_raw_angular_rate.i16bit[0]);
#endif
              break;

            default:
              /* Flush unused samples */
              printf("r");
              memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
              lsm6dso_fifo_out_raw_get(&dev_ctx, dummy.u8bit);
              break;
          }
          if (xl_flag && gy_flag) {
            xl_flag = false;
            gy_flag = false;
            cf_filter_apply(imu_data, sf_cf_data);
          }
        }

#ifdef DEBUG_PRINT
      printf("XL: %d, GY: %d\r\n", XLnum, GYnum);
#endif
        print_sf_cf_results(sf_cf_data);
      }
      xSemaphoreGive(i2cMutex);
    }
  }
  vTaskDelete(NULL);
}

static void x_get_inclinometer_task (void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1010);
  for (;;) {
    /* Send a notification to imu task, bringing it out of the
     Blocked state. */
    xTaskNotifyGive(xImuReadAndComputeH);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12);
  }
  vTaskDelete(NULL);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main (void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialise low-pass filters */
  for (uint8_t n = 0; n < 3; n++) {

    rc_filter_init(&lpf_xl_coeff[n], 139.0f, 0.01f);

    rc_filter_init(&lpf_gy_coeff[n], 839.0f, 0.01f);

  }

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;
  i2cMutex = xSemaphoreCreateMutex();
//  platform_delay(LSM6DSO_BOOT_TIME);
  lsm6dso_init();

  if (i2cMutex != NULL) {
    xTaskCreate(v_imu_read_and_compute_task, "Read IMU Data", configMINIMAL_STACK_SIZE, NULL, 1, &xImuReadAndComputeH);
  }

  xTaskCreate(x_get_inclinometer_task, "Periodically Get Inclination", configMINIMAL_STACK_SIZE, NULL, 1, &xGetInclinationH);
  vTaskStartScheduler();
  /* USER CODE END 2 */

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
void SystemClock_Config (void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Power Configuration
 * @retval None
 */
static void SystemPower_Config (void) {
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN PWR */
  /* USER CODE END PWR */
}

/**
 * @brief GPDMA1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPDMA1_Init (void) {

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init (void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F07BFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init (void) {

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  ICACHE_RegionConfigTypeDef pRegionConfig = { 0 };

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Configure and enable a region for memory remapping.
   */
  if (HAL_ICACHE_Disable() != HAL_OK) {
    Error_Handler();
  }
  pRegionConfig.BaseAddress = 0x10000000;
  pRegionConfig.RemapAddress = 0x60000000;
  pRegionConfig.Size = ICACHE_REGIONSIZE_2MB;
  pRegionConfig.TrafficRoute = ICACHE_MASTER1_PORT;
  pRegionConfig.OutputBurstType = ICACHE_OUTPUT_BURST_WRAP;
  if (HAL_ICACHE_EnableRemapRegion(_NULL, &pRegionConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Enable instruction cache in 1-way (direct mapped cache)
   */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init (void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
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
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init (void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBn_Pin | LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_DBn_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin | LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void print_sf_cf_results (struct cf_data *result) {
  sprintf((char*) tx_buffer, "%.3f\t%.3f\r\n", result->phiHat_deg, result->theta_hat_rad);
//  sprintf((char*) tx_buffer, "%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\r\n",result->phiHat_acc,result->thetaHat_acc,result->p_rps,result->q_rps,result->r_rps,result->phiDot*RAD2DEG,result->thetaDot*RAD2DEG,result->phiHat_rad*RAD2DEG,result->thetaHat_rad*RAD2DEG);
  tx_com(tx_buffer, strlen((const char*) tx_buffer));
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write (void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read (void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com (uint8_t *tx_buffer, uint16_t len) {
  HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay (uint32_t ms) {
  HAL_Delay(ms);
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
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
void Error_Handler (void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
