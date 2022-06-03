/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TIMEOUT 200

#define CELLS_PER_SEGMENT 16
#define NUM_OF_SEGMENTS 6
#define NUM_OF_CELLS (CELLS_PER_SEGMENT * NUM_OF_SEGMENTS)
#define MIN_CELL_VOLTAGE 2.5
#define MAX_CELL_VOLTAGE 4.2
#define MIN_SEGMENT_VOLTAGE (MIN_CELL_VOLTAGE * CELLS_PER_SEGMENT)
#define MAX_SEGMENT_VOLTAGE (MAX_CELL_VOLTAGE * CELLS_PER_SEGMENT)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
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
  MX_ADC_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	//turn on green LED. If this turns red, there's been an error
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

	//Set SPI1 bus to talk to segment #1
	//Keep in mind that NSS is active low - so RESET turns it on, SET turns it off
	HAL_GPIO_WritePin(SPI_NSS_1_GPIO_Port, SPI_NSS_1_Pin, GPIO_PIN_RESET);
	//Set SPI1 bus to not talk to all the other segments
	HAL_GPIO_WritePin(SPI_NSS_2_GPIO_Port, SPI_NSS_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_NSS_3_GPIO_Port, SPI_NSS_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_NSS_4_GPIO_Port, SPI_NSS_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_NSS_5_GPIO_Port, SPI_NSS_5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_NSS_6_GPIO_Port, SPI_NSS_6_Pin, GPIO_PIN_SET);

	uint16_t buffer_in[16] = {0};

	CAN_TxHeaderTypeDef tx_header;
	tx_header.IDE = CAN_ID_STD; //we're using the Standard ID, not extended
	tx_header.StdId = 0;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 3;

	uint8_t tx_data[8];
	tx_data[0] = 0xC0;
	tx_data[1] = 0xFF;
	tx_data[2] = 0xEE;

	//Configure the CAN filter.
	//This filters out all the can messages we don't want to listen to.
	CAN_FilterTypeDef can_filter_config;
	can_filter_config.FilterActivation = CAN_FILTER_DISABLE;
	can_filter_config.SlaveStartFilterBank = 10;
	can_filter_config.FilterBank = 0;
	can_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_config.FilterIdHigh = 0x0<<5;
	can_filter_config.FilterIdLow = 0x0;
	can_filter_config.FilterMaskIdHigh = 0x0<<5;
	can_filter_config.FilterMaskIdLow = 0x0000;

	//enable CANbus filter config
	if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK) {
		Error_Handler();
	}

	//start CANbus port
	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	//tells us which mailbox an outgoing message went to
	uint32_t tx_mailbox;

	for (int i = 0; i < 4; i++) {
		if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
			Error_Handler();
		}
		while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {asm("NOP");}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	do {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//Get data from each of the BMS segments.
		//Right now we're just going to use one segment, but soon we'll use all 6.
		uint8_t sync = 'S';
		if (HAL_SPI_Transmit(&hspi1, &sync, 1, SPI_TIMEOUT) != HAL_OK) {
			Error_Handler();
		}

//		if (HAL_SPI_Receive(&hspi1, (uint8_t*) buffer_in, 12 * sizeof(uint16_t), 1000) != HAL_OK) {
//			Error_Handler();
//		}

		for (int i = 0; i < 12; i++) {
			if (HAL_SPI_Receive(&hspi1, (uint8_t *) &buffer_in[i], sizeof(uint8_t), 400) != HAL_OK) {
				Error_Handler();
			}
		}

		if (HAL_SPI_Receive(&hspi1, (uint8_t*) &buffer_in[12], 4 * sizeof(uint16_t), 1000) != HAL_OK) {
			Error_Handler();
		}

		//check overall voltage of the segment and each cell's voltage
		float avg_cell_voltage = 0;
		float segment_voltages[16] = {0};
		for (int i = 0; i < 16; i++) {
			//convert adc number into voltage
//			segment_voltages[i] = buffer_in[i] * 3.3f / 4096;
//			avg_cell_voltage += (segment_voltages[i] / CELLS_PER_SEGMENT);
			//send ith cell voltage over CANbus
			tx_data[0] = (0 << 4) | i; //first nibble is segment #, second nibble is cell #
			tx_data[1] = (uint8_t) (buffer_in[i] >> 8); //high byte of cell voltage
			tx_data[2] = (uint8_t) buffer_in[i]; //low byte of cell voltage
			if (buffer_in[i] == 0) {
				Error_Handler();
			}
			while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {asm("NOP");}
			if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
				Error_Handler();
			}
		}

		while (HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox)) {asm("NOP");}
//		if (avg_cell_voltage < MIN_SEGMENT_VOLTAGE) {
//			//open AIR-
//			HAL_GPIO_WritePin(AIR__ctrl_GPIO_Port, AIR__ctrl_Pin, GPIO_PIN_RESET);
//			//Error_Handler();
//		} else if (avg_cell_voltage > MAX_SEGMENT_VOLTAGE) {
//			//open AIR-
//			HAL_GPIO_WritePin(AIR__ctrl_GPIO_Port, AIR__ctrl_Pin, GPIO_PIN_RESET);
//			//Error_Handler();
//		}

		//TODO: check temps

		//Accumulator safe, so we turn on AIR- (or leave it on, if it was already set)
		HAL_GPIO_WritePin(AIR__ctrl_GPIO_Port, AIR__ctrl_Pin, GPIO_PIN_SET);
		//turn on Green LED (LED2)
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

		//TODO: check the state of the charging pin
		//TODO: if charging, cutoff and do a balancing session at the end.

	} while (0);
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
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, SOC_gauge_Pin|fan_enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BMS_fault_Pin|precharge_ctrl_Pin|AIR__ctrl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_NSS_1_Pin|SPI_NSS_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_NSS_3_Pin|SPI_NSS_4_Pin|SPI_NSS_5_Pin|SPI_NSS_6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SOC_gauge_Pin fan_enable_Pin */
  GPIO_InitStruct.Pin = SOC_gauge_Pin|fan_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : reset_Pin charger_Pin */
  GPIO_InitStruct.Pin = reset_Pin|charger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_fault_Pin SPI_NSS_1_Pin precharge_ctrl_Pin AIR__ctrl_Pin
                           SPI_NSS_2_Pin */
  GPIO_InitStruct.Pin = BMS_fault_Pin|SPI_NSS_1_Pin|precharge_ctrl_Pin|AIR__ctrl_Pin
                          |SPI_NSS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin SPI_NSS_3_Pin SPI_NSS_4_Pin
                           SPI_NSS_5_Pin SPI_NSS_6_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|SPI_NSS_3_Pin|SPI_NSS_4_Pin
                          |SPI_NSS_5_Pin|SPI_NSS_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	//turn off green LED
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	//turn on red LED (LED1)
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
