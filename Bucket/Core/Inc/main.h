/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define userLed_Pin GPIO_PIN_13
#define userLed_GPIO_Port GPIOC
#define AIN1_LB_Pin GPIO_PIN_14
#define AIN1_LB_GPIO_Port GPIOC
#define AIN2_LB_Pin GPIO_PIN_15
#define AIN2_LB_GPIO_Port GPIOC
#define encoderPhasA_RB_Pin GPIO_PIN_0
#define encoderPhasA_RB_GPIO_Port GPIOA
#define encoderPhasB_RB_Pin GPIO_PIN_1
#define encoderPhasB_RB_GPIO_Port GPIOA
#define current_LB_Pin GPIO_PIN_4
#define current_LB_GPIO_Port GPIOA
#define AIN1_LF_Pin GPIO_PIN_5
#define AIN1_LF_GPIO_Port GPIOA
#define AIN2_LF_Pin GPIO_PIN_6
#define AIN2_LF_GPIO_Port GPIOA
#define current_RB_Pin GPIO_PIN_7
#define current_RB_GPIO_Port GPIOA
#define current_LF_Pin GPIO_PIN_0
#define current_LF_GPIO_Port GPIOB
#define current_RF_Pin GPIO_PIN_1
#define current_RF_GPIO_Port GPIOB
#define AIN2_RB_Pin GPIO_PIN_12
#define AIN2_RB_GPIO_Port GPIOB
#define AIN1_RB_Pin GPIO_PIN_13
#define AIN1_RB_GPIO_Port GPIOB
#define AIN2_RF_Pin GPIO_PIN_14
#define AIN2_RF_GPIO_Port GPIOB
#define AIN1_RF_Pin GPIO_PIN_15
#define AIN1_RF_GPIO_Port GPIOB
#define pwmMotor_RB_Pin GPIO_PIN_8
#define pwmMotor_RB_GPIO_Port GPIOA
#define pwmMotor_LB_Pin GPIO_PIN_9
#define pwmMotor_LB_GPIO_Port GPIOA
#define pwmMotor_RF_Pin GPIO_PIN_10
#define pwmMotor_RF_GPIO_Port GPIOA
#define pwmMotor_LF_Pin GPIO_PIN_11
#define pwmMotor_LF_GPIO_Port GPIOA
#define encoderPhasA_LB_Pin GPIO_PIN_4
#define encoderPhasA_LB_GPIO_Port GPIOB
#define encoderPhasB_LB_Pin GPIO_PIN_5
#define encoderPhasB_LB_GPIO_Port GPIOB
#define encoderPhasA_RF_Pin GPIO_PIN_6
#define encoderPhasA_RF_GPIO_Port GPIOB
#define encoderPhasB_RF_Pin GPIO_PIN_7
#define encoderPhasB_RF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
