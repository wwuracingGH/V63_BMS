/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SOC_gauge_Pin GPIO_PIN_0
#define SOC_gauge_GPIO_Port GPIOF
#define fan_enable_Pin GPIO_PIN_1
#define fan_enable_GPIO_Port GPIOF
#define current_sense_low_Pin GPIO_PIN_0
#define current_sense_low_GPIO_Port GPIOA
#define current_sense_high_Pin GPIO_PIN_1
#define current_sense_high_GPIO_Port GPIOA
#define reset_Pin GPIO_PIN_2
#define reset_GPIO_Port GPIOA
#define BMS_fault_Pin GPIO_PIN_3
#define BMS_fault_GPIO_Port GPIOA
#define SPI_NSS_1_Pin GPIO_PIN_4
#define SPI_NSS_1_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_0
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOB
#define charger_Pin GPIO_PIN_8
#define charger_GPIO_Port GPIOA
#define precharge_ctrl_Pin GPIO_PIN_9
#define precharge_ctrl_GPIO_Port GPIOA
#define AIR__ctrl_Pin GPIO_PIN_10
#define AIR__ctrl_GPIO_Port GPIOA
#define SPI_NSS_2_Pin GPIO_PIN_15
#define SPI_NSS_2_GPIO_Port GPIOA
#define SPI_NSS_3_Pin GPIO_PIN_3
#define SPI_NSS_3_GPIO_Port GPIOB
#define SPI_NSS_4_Pin GPIO_PIN_4
#define SPI_NSS_4_GPIO_Port GPIOB
#define SPI_NSS_5_Pin GPIO_PIN_5
#define SPI_NSS_5_GPIO_Port GPIOB
#define SPI_NSS_6_Pin GPIO_PIN_6
#define SPI_NSS_6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
