/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define Torque_Rate_Pin GPIO_PIN_0
#define Torque_Rate_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_1
#define APPS2_GPIO_Port GPIOA
#define RF_Shock_Length_Pin GPIO_PIN_2
#define RF_Shock_Length_GPIO_Port GPIOA
#define LF_Shock_Length_Pin GPIO_PIN_3
#define LF_Shock_Length_GPIO_Port GPIOA
#define Steering_Angle_Pin GPIO_PIN_4
#define Steering_Angle_GPIO_Port GPIOA
#define R_Brake_Pressure_Pin GPIO_PIN_5
#define R_Brake_Pressure_GPIO_Port GPIOA
#define F_Brake_Pressure_Pin GPIO_PIN_6
#define F_Brake_Pressure_GPIO_Port GPIOA
#define Pitot_Tube_Pin GPIO_PIN_7
#define Pitot_Tube_GPIO_Port GPIOA
#define APPS1_Pin GPIO_PIN_0
#define APPS1_GPIO_Port GPIOB
#define RTD_Button_Pin GPIO_PIN_1
#define RTD_Button_GPIO_Port GPIOB
#define RTD_Button_EXTI_IRQn EXTI0_1_IRQn
#define Fault_Light_Pin GPIO_PIN_8
#define Fault_Light_GPIO_Port GPIOA
#define RTD_Light_Pin GPIO_PIN_9
#define RTD_Light_GPIO_Port GPIOA
#define Traction_Control_Light_Pin GPIO_PIN_10
#define Traction_Control_Light_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define BSPD_Fault_Pin GPIO_PIN_4
#define BSPD_Fault_GPIO_Port GPIOB
#define RTD_Buzzer_Pin GPIO_PIN_5
#define RTD_Buzzer_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
