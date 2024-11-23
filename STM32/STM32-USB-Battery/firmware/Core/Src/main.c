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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_ina219_interface.h"
#include "usbd_cdc_if.h"
#include "slip.h"
#include "stm32_msg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t data[16];
} bcu_usb_msg;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
osThreadId usbRxTaskHandle;
osThreadId usbTxTaskHandle;
osMessageQId usbRxQueueHandle;
uint8_t usbRxQueueBuffer[ 16 * sizeof( bcu_usb_msg ) ];
osStaticMessageQDef_t usbRxQueueControlBlock;
osMessageQId usbTxQueueHandle;
uint8_t usbTxQueueBuffer[ 16 * sizeof( stm32_cmd_msg_t ) ];
osStaticMessageQDef_t usbTxQueueControlBlock;
osSemaphoreId usbTxAvailableHandle;
osStaticSemaphoreDef_t usbTxAvailableControlBlock;
/* USER CODE BEGIN PV */
static ina219_handle_t gs_handle; /**< ina219 handle */
bm_data_t bm_data = { 0, 0, 0 };
RTC_TimeTypeDef gsTime = { 0 };
RTC_DateTypeDef gsDate = { 0 };

RTC_TimeTypeDef guTime = { 0 };
RTC_DateTypeDef guDate = { 0 };
uint8_t dateReadyToSet = 0;
uint32_t upTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void UsbReceive(void const * argument);
void UsbTransmit(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t SYS_handle_request(stm32_cmd_msg_t *request_msg,
		stm32_cmd_msg_t *reply_msg) {
	uint8_t ret = STM32_CMD_STATUS_REPLY_READY;

	switch (request_msg->cmd_addr) {

	case STM_MSG_SYS_DATEH:
		reply_msg->data = ((gsDate.Month << 8) | gsDate.WeekDay);
		break;

	case STM_MSG_SYS_DATEL:
		reply_msg->data = ((gsDate.Year << 8) | gsDate.Date);
		break;

	case STM_MSG_SYS_TIMEH:
		reply_msg->data = ((gsTime.Minutes << 8) | gsTime.Hours);
		break;

	case STM_MSG_SYS_TIMEL:
		reply_msg->data = ((gsTime.TimeFormat << 8) | gsTime.Seconds);
		break;

	case STM_MSG_SYS_UPTIME:

		break;

	default:
		ret = STM32_CMD_STATUS_UNKNOWN_CMD;
		break;

	}

	return ret;
}

uint8_t SYS_handle_command(stm32_cmd_msg_t *cmd_msg) {
	uint8_t ret = STM32_CMD_STATUS_NO_REPLY;

	switch (cmd_msg->cmd_addr) {

	case STM_MSG_SYS_DATEH:
		guDate.Month = (uint8_t) (cmd_msg->data >> 8);
		guDate.WeekDay = (uint8_t) (cmd_msg->data & 0x00ff);
		break;

	case STM_MSG_SYS_DATEL:
		guDate.Year = (uint8_t) (cmd_msg->data >> 8);
		guDate.Date = (uint8_t) (cmd_msg->data & 0x00ff);
		break;

	case STM_MSG_SYS_TIMEH:
		guTime.Minutes = (uint8_t) (cmd_msg->data >> 8);
		guTime.Hours = (uint8_t) (cmd_msg->data & 0x00ff);
		// bm_data.control = cmd_msg->data;
		break;

	case STM_MSG_SYS_TIMEL:
		// bm_data.control = cmd_msg->data;
		bool setDateTime = cmd_msg->data & 0x8000;
		guTime.TimeFormat = (uint8_t) ((cmd_msg->data >> 8) & 0x0001);
		guTime.Seconds = (uint8_t) (cmd_msg->data & 0x00ff);
		if (setDateTime) {
			guTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			guTime.StoreOperation = RTC_STOREOPERATION_SET;
			if (HAL_RTC_SetTime(&hrtc, &guTime, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}

			if (HAL_RTC_SetDate(&hrtc, &guDate, RTC_FORMAT_BIN) != HAL_OK) {
				Error_Handler();
			}
		}
		break;

	default:
		ret = STM32_CMD_STATUS_UNKNOWN_CMD;
		break;
	}
	return ret;
}

/**
 * @brief      Handle request to read an INA219 register
 *
 * @param      request_msg  The request message
 * @param      reply_msg    The reply message
 *
 * @return     Status
 */
uint8_t INA219_handle_request(stm32_cmd_msg_t *request_msg,
		stm32_cmd_msg_t *reply_msg) {
	uint8_t status;
	uint8_t ret = STM32_CMD_STATUS_REPLY_READY;
	status = ina219_get_reg(&gs_handle, request_msg->cmd_addr,
			&reply_msg->data);
	if (status != 0) {
		ret = STM32_CMD_STATUS_REPLY_ERROR;
	}
	return ret;
}

/**
 * @brief      Handle command to set an INA219 register
 *
 * @param      cmd_msg  The command message
 *
 * @return     Status
 */
uint8_t INA219_handle_command(stm32_cmd_msg_t *cmd_msg) {
	uint8_t status;
	uint8_t ret = STM32_CMD_STATUS_NO_REPLY;
	status = ina219_set_reg(&gs_handle, (uint8_t) cmd_msg->cmd_addr,
			cmd_msg->data);
	if (status != 0) {
		ret = STM32_CMD_STATUS_ERROR;
	}
	return ret;
}

/**
 * @brief      Handle request to read a Battery Charger register
 *
 * @param      request_msg  The request message
 * @param      reply_msg    The reply message
 *
 * @return     Status
 */
uint8_t BC_handle_request(stm32_cmd_msg_t *request_msg,
		stm32_cmd_msg_t *reply_msg) {

	uint8_t ret = STM32_CMD_STATUS_REPLY_READY;

	switch (request_msg->cmd_addr) {

	case STM_MSG_BC_STAT:
		reply_msg->data = bm_data.stat;
		break;

	case STM_MSG_BC_CONTROL:
		reply_msg->data = bm_data.control;
		break;

	case STM_MSG_BC_VSUPPLY:
		reply_msg->data = bm_data.vsupply;
		break;

	default:
		ret = STM32_CMD_STATUS_UNKNOWN_CMD;
		break;
	}
	return ret;
}

/**
 * @brief      Handle command to set a Battery Charger register
 *
 * @param      cmd_msg  The command message
 *
 * @return     Status
 */
uint8_t BC_handle_command(stm32_cmd_msg_t *cmd_msg) {
	uint8_t ret = STM32_CMD_STATUS_NO_REPLY;

	switch (cmd_msg->cmd_addr) {

	case STM_MSG_BC_CONTROL:
		bm_data.control = cmd_msg->data;
		break;

	default:
		ret = STM32_CMD_STATUS_UNKNOWN_CMD;
		break;
	}
	return ret;
}

/**
 * @brief      USB command handler table
 */

static stm32_msg_handler_t usb_command_handler[] = { { STM32_MSG_GROUP_SYS,
		&SYS_handle_request, &SYS_handle_command }, { STM32_MSG_GROUP_INA219,
		&INA219_handle_request, &INA219_handle_command }, { STM32_MSG_GROUP_BC,
		&BC_handle_request, &BC_handle_command } };

#define STM32_MSG_GROUP_MAX ((sizeof(usb_command_handler) / sizeof(stm32_msg_handler_t)) - 1)

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t ina219_hw_interface_iic_init(void) {
	return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t ina219_hw_interface_iic_deinit(void) {
	return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success;
 *             - 1 read failed
 * @note       none
 */
uint8_t ina219_hw_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
		uint16_t len) {
	HAL_StatusTypeDef halStatus;

	halStatus = HAL_I2C_Mem_Read(&hi2c3, addr, reg, 1, buf, len, 5000);

	return (halStatus != HAL_OK);
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ina219_hw_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
		uint16_t len) {
	HAL_StatusTypeDef halStatus;

	halStatus = HAL_I2C_Mem_Write(&hi2c3, addr, reg, 1, buf, len, 5000);

	return (halStatus != HAL_OK);
}

uint8_t ina219_basic_init(ina219_address_t addr_pin, double r) {
	uint8_t res;
	uint16_t calibration;

	/* link interface function */
	DRIVER_INA219_LINK_INIT(&gs_handle, ina219_handle_t);
	DRIVER_INA219_LINK_IIC_INIT(&gs_handle, ina219_hw_interface_iic_init);
	DRIVER_INA219_LINK_IIC_DEINIT(&gs_handle, ina219_hw_interface_iic_deinit);
	DRIVER_INA219_LINK_IIC_READ(&gs_handle, ina219_hw_interface_iic_read);
	DRIVER_INA219_LINK_IIC_WRITE(&gs_handle, ina219_hw_interface_iic_write);
	DRIVER_INA219_LINK_DELAY_MS(&gs_handle, ina219_interface_delay_ms);
	DRIVER_INA219_LINK_DEBUG_PRINT(&gs_handle, ina219_interface_debug_print);

	/* set addr pin */
	res = ina219_set_addr_pin(&gs_handle, addr_pin);
	if (res != 0) {
		printf("ina219: set addr pin failed.\n");

		return 1;
	}

	/* set the r */
	res = ina219_set_resistance(&gs_handle, r);
	if (res != 0) {
		printf("ina219: set resistance failed.\n");

		return 1;
	}

	/* init */
	res = ina219_init(&gs_handle);
	if (res != 0) {
		printf("ina219: init failed.\n");

		return 1;
	}

	/* set bus voltage range */
	res = ina219_set_bus_voltage_range(&gs_handle,
			INA219_BUS_VOLTAGE_RANGE_16V);
	if (res != 0) {
		printf("ina219: set bus voltage range failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* set bus voltage adc mode */
	res = ina219_set_bus_voltage_adc_mode(&gs_handle,
			INA219_ADC_MODE_12_BIT_16_SAMPLES);
	if (res != 0) {
		printf("ina219: set bus voltage adc mode failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* set shunt voltage adc mode */
	res = ina219_set_shunt_voltage_adc_mode(&gs_handle,
			INA219_ADC_MODE_12_BIT_16_SAMPLES);
	if (res != 0) {
		printf("ina219: set shunt voltage adc mode failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* set shunt bus voltage continuous */
	res = ina219_set_mode(&gs_handle, INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);
	if (res != 0) {
		printf("ina219: set mode failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* set pga */
	res = ina219_set_pga(&gs_handle, INA219_PGA_320_MV);
	if (res != 0) {
		printf("ina219: set pga failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* calculate calibration */
	res = ina219_calculate_calibration(&gs_handle, (uint16_t*) &calibration);
	if (res != 0) {
		printf("ina219: calculate calibration failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	/* set calibration */
	res = ina219_set_calibration(&gs_handle, calibration);
	if (res != 0) {
		printf("ina219: set calibration failed.\n");
		(void) ina219_deinit(&gs_handle);

		return 1;
	}

	return 0;
}

uint8_t ina219_basic_read(float *mV, float *mA, float *mW) {
	uint8_t res;
	int16_t s_raw;
	uint16_t u_raw;

	/* read bus voltage */
	res = ina219_read_bus_voltage(&gs_handle, (uint16_t*) &u_raw, mV);
	if (res != 0) {
		return 1;
	}

	/* read current */
	res = ina219_read_current(&gs_handle, (int16_t*) &s_raw, mA);
	if (res != 0) {
		return 1;
	}

	/* read power */
	res = ina219_read_power(&gs_handle, (uint16_t*) &u_raw, mW);
	if (res != 0) {
		return 1;
	}

	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t status;

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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	// DWT_Delay_Init();
	status = ina219_basic_init(INA219_ADDRESS_0, 0.1);

	printf("INA219 Status = %d\n", status);

	HAL_GPIO_WritePin(BC_SEL_GPIO_Port, BC_SEL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BC_PROG2_GPIO_Port, BC_PROG2_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(BC_CE_GPIO_Port, BC_CE_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of usbTxAvailable */
  osSemaphoreStaticDef(usbTxAvailable, &usbTxAvailableControlBlock);
  usbTxAvailableHandle = osSemaphoreCreate(osSemaphore(usbTxAvailable), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of usbRxQueue */
  osMessageQStaticDef(usbRxQueue, 16, bcu_usb_msg, usbRxQueueBuffer, &usbRxQueueControlBlock);
  usbRxQueueHandle = osMessageCreate(osMessageQ(usbRxQueue), NULL);

  /* definition and creation of usbTxQueue */
  osMessageQStaticDef(usbTxQueue, 16, stm32_cmd_msg_t, usbTxQueueBuffer, &usbTxQueueControlBlock);
  usbTxQueueHandle = osMessageCreate(osMessageQ(usbTxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usbRxTask */
  osThreadDef(usbRxTask, UsbReceive, osPriorityNormal, 0, 512);
  usbRxTaskHandle = osThreadCreate(osThread(usbRxTask), NULL);

  /* definition and creation of usbTxTask */
  osThreadDef(usbTxTask, UsbTransmit, osPriorityNormal, 0, 512);
  usbTxTaskHandle = osThreadCreate(osThread(usbTxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if 0
  while (1)
  {

	  uint8_t regStatus;
	  uint8_t reg;
	  stm32_cmd_msg_t unpackedMsg;

	  memset(&unpackedMsg, 0x00, sizeof(unpackedMsg));

	  unpackedMsg.cmd_addr_group = STM32_MSG_DATA_FLAG | STM32_MSG_GROUP_INA219;

	  for (reg = 0; reg <= STM_MSG_INA219_CALIBRATION; reg++) {

		  unpackedMsg.cmd_addr = reg;

		  regStatus =  ina219_get_reg(&gs_handle, reg, &unpackedMsg.read_data);

		  if (!regStatus)
		  {
			  residue = sizeof(slipTxBuffer);
			  slipStatus =  slipSlipData((uint8_t*)&unpackedMsg, sizeof(unpackedMsg), slipTxBuffer, &residue);
			  CDC_Transmit_FS(slipTxBuffer, residue);
		  }



		  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
		  HAL_Delay(500);

	  }


    HAL_Delay(500);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#endif
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 100-1;
  hrtc.Init.SynchPrediv = 10000-1;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BC_CE_Pin|BC_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BC_PROG2_GPIO_Port, BC_PROG2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BC_STAT_PG_Pin */
  GPIO_InitStruct.Pin = BC_STAT_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BC_STAT_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BC_STAT_2_Pin */
  GPIO_InitStruct.Pin = BC_STAT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BC_STAT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BC_STAT_1_Pin */
  GPIO_InitStruct.Pin = BC_STAT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BC_STAT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BC_CE_Pin BC_SEL_Pin BC_PROG2_Pin */
  GPIO_InitStruct.Pin = BC_CE_Pin|BC_SEL_Pin|BC_PROG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef BATTERY_DEBUG_USB
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len); return len;
}
#else
# ifdef BATTERY_DEBUG_SWO
int __io_putchar(int ch) {
	// Write character to ITM ch.0
	ITM_SendChar(ch);
	return (ch);
}
# endif
#endif

void UsbTxCompleteCallBack(uint8_t *Buf, uint32_t Len) {
	UNUSED(Buf);
	UNUSED(Len);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(usbTxAvailableHandle, &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UsbRxCallBack(uint8_t *Buf, uint32_t Len) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Need to apply some back pressure to the client if the queue is filling up...

	if (Len <= 16) {
		if (xQueueIsQueueFullFromISR(usbRxQueueHandle) == pdFALSE) {

			xQueueSendToBackFromISR(usbRxQueueHandle, Buf,
					&xHigherPriorityTaskWoken);
		} else {
			printf("usbRxQueue full");
		}

		if (xHigherPriorityTaskWoken) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	return;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	GPIO_PinState pinState;
	uint16_t lastControl = bm_data.control;
	uint32_t AD_RES[6] = { 0 };
	/* Infinite loop */
	HAL_ADC_Start_DMA(&hadc1, &AD_RES[0], 3);
	for (;;) {

		bm_data.stat = 0;

		bm_data.vsupply = (uint16_t) AD_RES[0];

		// printf("vs %d %ld\n", bm_data.vsupply, HAL_ADC_GetError(&hadc1));

		pinState = HAL_GPIO_ReadPin(BC_STAT_PG_GPIO_Port, BC_STAT_PG_Pin);
		bm_data.stat |= ((pinState == GPIO_PIN_RESET) << 0);

		pinState = HAL_GPIO_ReadPin(BC_STAT_2_GPIO_Port, BC_STAT_2_Pin);
		bm_data.stat |= ((pinState == GPIO_PIN_RESET) << 1);

		pinState = HAL_GPIO_ReadPin(BC_STAT_1_GPIO_Port, BC_STAT_1_Pin);
		bm_data.stat |= ((pinState == GPIO_PIN_RESET) << 2);

		if (lastControl != bm_data.control) {
			HAL_GPIO_WritePin(BC_SEL_GPIO_Port, BC_SEL_Pin,
					bm_data.control & 0x0001 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BC_PROG2_GPIO_Port, BC_PROG2_Pin,
					bm_data.control & 0x0002 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BC_CE_GPIO_Port, BC_CE_Pin,
					bm_data.control & 0x0004 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			lastControl = bm_data.control;
		}

		HAL_StatusTypeDef res;

		res = HAL_RTC_GetTime(&hrtc, &gsTime, RTC_FORMAT_BIN);
		if (res != HAL_OK) {
			printf("HAL_RTC_GetTime failed: %d\r\n", res);
			return;
		}

		res = HAL_RTC_GetDate(&hrtc, &gsDate, RTC_FORMAT_BIN);
		if (res != HAL_OK) {
			printf("HAL_RTC_GetDate failed: %d\r\n", res);
			return;
		}

		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UsbReceive */
/**
 * @brief Function implementing the usbRxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UsbReceive */
void UsbReceive(void const * argument)
{
  /* USER CODE BEGIN UsbReceive */
	/* Infinite loop */
	bcu_usb_msg rxMessage;
	stm32_cmd_msg_t cmdMsg;
	slip_context_t context;
	slip_status_t status;
	for (;;) {
		if (xQueueReceive(usbRxQueueHandle, &(rxMessage),
				(TickType_t) 10)!= pdPASS) {
			// No message ready
			osDelay(1);
		} else {
			// Unslip the message
			slipSetRxBuffer(&context, (uint8_t*) &cmdMsg, sizeof(cmdMsg));
			status = slipUnslipData(&context, (uint8_t*) &rxMessage,
					sizeof(rxMessage));

			if (status != FRAME_COMPLETE) {
				printf("Incomplete Message %d \n", status);
			} else {
				if (xQueueSendToBack(usbTxQueueHandle, (uint8_t* ) &cmdMsg,
						0) == pdFALSE) {
					printf("Tx Q full\n");
				}
			}

		}

	}
  /* USER CODE END UsbReceive */
}

/* USER CODE BEGIN Header_UsbTransmit */
/**
 * @brief Function implementing the usbTxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UsbTransmit */
void UsbTransmit(void const * argument)
{
  /* USER CODE BEGIN UsbTransmit */
	/* Infinite loop */
	stm32_cmd_msg_t usbMsg;
	stm32_cmd_msg_t replyMsg;
	uint8_t cmdAddrGroup;
	uint8_t slipTxBuffer[32];
	slip_status_t slipStatus;
	uint32_t residue;
	uint8_t usbSendStatus;
	uint8_t cmdStatus;

	for (;;) {
		if (xQueueReceive(usbTxQueueHandle, &(usbMsg),
				(TickType_t) 10) != pdPASS) {
			// No message ready
			osDelay(1);
		} else {
			usbSendStatus = USBD_OK;

			// Check that the message group is valid
			cmdAddrGroup = (usbMsg.cmd_addr_group & ~STM32_MSG_DATA_FLAG);
			if (cmdAddrGroup <= STM32_MSG_GROUP_MAX) {
				if (((usbMsg.cmd_addr_group & STM32_MSG_DATA_FLAG) >> 7)) {
					// Execute the command, there is no response
					cmdStatus = usb_command_handler[cmdAddrGroup].command_msg(
							&usbMsg);
				} else {
					// Execute the command, expect a response
					replyMsg.cmd_addr_group = (usbMsg.cmd_addr_group
							| STM32_MSG_DATA_FLAG);
					replyMsg.cmd_addr = usbMsg.cmd_addr;

					cmdStatus = usb_command_handler[cmdAddrGroup].request_msg(
							&usbMsg, &replyMsg);
				}
			}

			if (cmdStatus == STM32_CMD_STATUS_REPLY_READY) {
				residue = sizeof(slipTxBuffer);
				slipStatus = slipSlipData((uint8_t*) &replyMsg,
						sizeof(replyMsg), slipTxBuffer, &residue);

				if (slipStatus != FRAME_COMPLETE) {
					printf("Frame Err : %d\n", slipStatus);
				} else {
					// Start sending the response
					usbSendStatus = CDC_Transmit_FS(slipTxBuffer, residue);

					if (usbSendStatus == USBD_OK) {
						// Wait here for the callback to complete
						if (xSemaphoreTake(usbTxAvailableHandle,
								10000) != pdTRUE) {
							printf("Timeout on semaphore\n");
						}
					}
				}

				if (usbSendStatus != USBD_OK) {
					printf("USB Err : %d\n", usbSendStatus);
				}
			}
		}
	}
  /* USER CODE END UsbTransmit */
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
  if (htim->Instance == TIM2) {
    upTime++;
    printf("%.2d %ld\n", gsTime.Seconds, upTime);
  }

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
