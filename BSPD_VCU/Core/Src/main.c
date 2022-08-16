/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TORQUE_RATE_KNOB 0
#define APPS2 1
#define RF_SHOCK_LENGTH 2
#define LF_SHOCK_LENGTH 3
#define STEERING_ANGLE 4
#define R_BRAKE_PRESS 5
#define F_BRAKE_PRESS 6
#define PITOT_TUBE 7
#define APPS1 8

#define APPS1_MIN 0
#define APPS1_MAX 3.3
#define APPS2_MIN 0
#define APPS2_MAX 3.3

//define limits of the "max torque" knob
#define MAX_TORQUE_MAX 200
#define MAX_TORQUE_MIN 10

//What 0V and 3.3V correspond to for brake pressure sensor
#define MAX_BRAKE_PRESS 1800
#define MIN_BRAKE_PRESS (-200)

//minimum brake pressure defined as "brakes on"
#define BRAKE_THRESH_PSI 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Sensor data will automatically be placed in here through DMA
volatile uint16_t raw_sensors[9];
uint8_t low_cell_temps[6];
uint8_t avg_cell_temps[6];
uint8_t high_cell_temps[6];

float get_sensor(uint8_t sensor) {
	return raw_sensors[sensor] * 3.3 / 4096;
}

bool apps_implausible;
bool apps_brake_implausible;

//convert voltage into a brake pressure value
uint16_t volt_to_brake_press(float sensor) {
	return sensor / 3.3 * (MAX_BRAKE_PRESS - MIN_BRAKE_PRESS) + MIN_BRAKE_PRESS;
}

//motor speed from motor controller
uint16_t motor_speed;

//Outgoing CANbus header data
CAN_TxHeaderTypeDef apps_header;
CAN_TxHeaderTypeDef shock_length_header;
CAN_TxHeaderTypeDef brake_press_header;
CAN_TxHeaderTypeDef bspd_diag_header;
CAN_TxHeaderTypeDef torque_command_header;

bool ready_to_drive;

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
  MX_ADC_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	ready_to_drive = false;

	apps_header.IDE = CAN_ID_STD;
	apps_header.StdId = 0x10;
	apps_header.RTR = CAN_RTR_DATA;
	apps_header.DLC = 8;

	shock_length_header.IDE = CAN_ID_STD;
	shock_length_header.StdId = 0x11;
	shock_length_header.RTR = CAN_RTR_DATA;
	shock_length_header.DLC = 8;

	brake_press_header.IDE = CAN_ID_STD;
	brake_press_header.StdId = 0x12;
	brake_press_header.RTR = CAN_RTR_DATA;
	brake_press_header.DLC = 8;

	bspd_diag_header.IDE = CAN_ID_STD;
	bspd_diag_header.StdId = 0x13;
	bspd_diag_header.RTR = CAN_RTR_DATA;
	bspd_diag_header.DLC = 1;

	torque_command_header.IDE = CAN_ID_STD;
	torque_command_header.StdId = 0x0C0;
	torque_command_header.RTR = CAN_RTR_DATA;
	torque_command_header.DLC = 8;

	//Configure the CAN filter.
	//This filters out all the can messages we don't want to listen to.
	CAN_FilterTypeDef can_filter_config;
	can_filter_config.FilterActivation = CAN_FILTER_DISABLE;
	can_filter_config.SlaveStartFilterBank = 13;
	can_filter_config.FilterBank = 13;
	can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_config.FilterIdHigh = (0x0A5)<<5;
	can_filter_config.FilterIdLow = 0x0;
	can_filter_config.FilterMaskIdHigh = (0x0A5)<<5;
	can_filter_config.FilterMaskIdLow = 0x0000;

	//enable CANbus filter config
	if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK) {
		Error_Handler();
	}

//	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
//		Error_Handler();
//	}

	//start CANbus port
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

	HAL_ADC_Start_DMA(&hadc, (uint32_t *) raw_sensors, 9);
	HAL_TIM_Base_Start_IT(&htim1);

	uint32_t tx_mailbox;
	uint8_t tx_test_data[8] = {0x00, 0x22, 0x44, 0x66, 0x88, 0xAA, 0xCC, 0xEE};
	if (HAL_CAN_AddTxMessage(&hcan, &torque_command_header, tx_test_data, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	do {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	} while (1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 95-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Fault_Light_Pin|RTD_Light_Pin|Traction_Control_Light_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|BSPD_Fault_Pin|RTD_Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RTD_Button_Pin */
  GPIO_InitStruct.Pin = RTD_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RTD_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Fault_Light_Pin RTD_Light_Pin Traction_Control_Light_Pin LED1_Pin */
  GPIO_InitStruct.Pin = Fault_Light_Pin|RTD_Light_Pin|Traction_Control_Light_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin BSPD_Fault_Pin RTD_Buzzer_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|BSPD_Fault_Pin|RTD_Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

//Callback function for RTD button. If brakes are held, initiates RTD sequence
void HAL_GPIO_EXTI_CALLBACK(uint16_t GPIO_pin) {
	if (GPIO_pin == RTD_Button_Pin) {
		if (!ready_to_drive && (volt_to_brake_press(get_sensor(F_BRAKE_PRESS)) > BRAKE_THRESH_PSI)) {
			//turn on RTD buzzer
			HAL_GPIO_WritePin(RTD_Buzzer_GPIO_Port, RTD_Buzzer_Pin, GPIO_PIN_SET);

			//start RTD timer
			//HAL_TIM_Base_Start_IT(&htim17);
		}
	}
}

//Callback for all timers.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) { //95ms oneshot timer for APPS plausibility
		if (apps_implausible) { //if apps is still implausible after 95 ms
			apps_implausible = 1;
		}
	}
	if (htim == &htim17) { //2000ms oneshot timer for RTD button
		//turn off buzzer
		HAL_GPIO_WritePin(RTD_Buzzer_GPIO_Port, RTD_Buzzer_Pin, GPIO_PIN_RESET);

		//go into ready to drive mode
		ready_to_drive = true;

		//start htim1, which starts the ADC DMA, which triggers the sending of sensor data to canbus
		HAL_ADC_Start_DMA(&hadc, (uint32_t *) raw_sensors, 9);
		HAL_TIM_Base_Start_IT(&htim1);
	}
}

//Sends adc info over CANbus.
//ADC events are automatically triggered by htim1.
//After the data gets DMA'd into RAM, this function is called.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	uint32_t tx_mailbox;
	uint8_t tx_test_data[8] = {0x00, 0x22, 0x44, 0x66, 0x88, 0xAA, 0xCC, 0xEE};
	if (HAL_CAN_AddTxMessage(&hcan, &torque_command_header, tx_test_data, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox) != 0) {}

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, HAL_GPIO_RESET);
	//get sensor values
	//ADC is constantly happening through DMA, so we don't have to manually scan in the values

	//APPS are values between 0 and 1
	float apps1 = (get_sensor(APPS1) - APPS1_MIN) / (APPS1_MAX - APPS1_MIN);
	float apps2 = (get_sensor(APPS2) - APPS2_MIN) / (APPS2_MAX - APPS2_MIN);
	float rf_shock_length = get_sensor(RF_SHOCK_LENGTH);
	float lf_shock_length = get_sensor(LF_SHOCK_LENGTH);
	float steering_angle = get_sensor(STEERING_ANGLE);
	uint16_t r_brake_pressure = volt_to_brake_press(get_sensor(R_BRAKE_PRESS));
	uint16_t f_brake_pressure = volt_to_brake_press(get_sensor(F_BRAKE_PRESS));
	//float pitot_tube = get_sensor(PITOT_TUBE);
	//float torque_rate_knob = get_sensor(0);
	//printf("%f\n", ((i / 4096.0 - 100/300) * (300 - 100)) + 100)
	float torque_rate_knob = (get_sensor(TORQUE_RATE_KNOB) / 4096.0 - (MAX_TORQUE_MIN * 1.0 /MAX_TORQUE_MAX))
					* (MAX_TORQUE_MAX - MAX_TORQUE_MIN) + MAX_TORQUE_MIN;

	//APPS plausibility check
	if (fabs(apps1 - apps2) > 0.1) {
		apps_implausible = 2;
	} else if (apps_implausible == 1) {
		apps_implausible = false;
	}

	//APPS/brake pedal plausibility check
	if ((apps1 > .25 || apps2 > .25)
			&& (f_brake_pressure > BRAKE_THRESH_PSI || r_brake_pressure > BRAKE_THRESH_PSI)) {
		apps_brake_implausible = true;
	}

	//calculate torque_request
	int16_t torque_request = ready_to_drive * ((apps1 + apps2) / 2) * torque_rate_knob;

	if (apps_implausible == 1) {
		torque_request = 0;
	}

	//create CANbus buffers
	uint32_t apps[2] = {apps1, apps2};
	uint32_t shock_lengths[2] = {lf_shock_length, rf_shock_length};
	uint16_t brake_steering[4] = {f_brake_pressure, r_brake_pressure, ((uint32_t) steering_angle) & 0xffff, ((uint32_t)steering_angle) >> 16};
	uint8_t torque_command[8] =
	{
			(uint8_t) torque_request, (uint8_t) (torque_request >> 8), //torque command (bit bashing used to force the bytes into the right order)
			0x00, 0x00,//speed command (not used)
			0x01, //Hard coded to always go forwards
			ready_to_drive | ((!ready_to_drive) << 1) | (0 << 2), //inverter enable | discharge enable
			0x00, 0x00 //commanded torque limit (0 uses default values)
	};

	//while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 2) {asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");}

	//transmit sensor data to CANbus.
	//while loops make it wait for can message to be sent before pushing another one
	if (HAL_CAN_AddTxMessage(&hcan, &apps_header, (uint8_t *) apps, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {HAL_Delay(1);}

	if (HAL_CAN_AddTxMessage(&hcan, &shock_length_header, (uint8_t *) shock_lengths, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {HAL_Delay(1);}

	if (HAL_CAN_AddTxMessage(&hcan, &brake_press_header, (uint8_t *) brake_steering, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {HAL_Delay(1);}

	if (HAL_CAN_AddTxMessage(&hcan, &torque_command_header, (uint8_t *) torque_command, &tx_mailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {HAL_Delay(1);}
}

//CANbus RX callback for getting motor speed info
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
		Error_Handler();
	} else {
		//if we're getting the motor position info packet, update the motor_speed value
		if (rx_header.StdId == 0x0A5) {
			motor_speed = rx_data[2] | (rx_data[3] << 8);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
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
