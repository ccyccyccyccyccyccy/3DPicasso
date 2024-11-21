/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern volatile int drawable;
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
#define K2_Pin GPIO_PIN_13
#define K2_GPIO_Port GPIOC
#define K1_Pin GPIO_PIN_0
#define K1_GPIO_Port GPIOA
#define K1_EXTI_IRQn EXTI0_IRQn
#define X_Pin GPIO_PIN_2
#define X_GPIO_Port GPIOA
#define XA3_Pin GPIO_PIN_3
#define XA3_GPIO_Port GPIOA
#define XA4_Pin GPIO_PIN_4
#define XA4_GPIO_Port GPIOA
#define XA5_Pin GPIO_PIN_5
#define XA5_GPIO_Port GPIOA
#define board_LED_Pin GPIO_PIN_0
#define board_LED_GPIO_Port GPIOB
#define board_LEDB1_Pin GPIO_PIN_1
#define board_LEDB1_GPIO_Port GPIOB
#define up_down_Pin GPIO_PIN_10
#define up_down_GPIO_Port GPIOB
#define y_Pin GPIO_PIN_12
#define y_GPIO_Port GPIOB
#define yB13_Pin GPIO_PIN_13
#define yB13_GPIO_Port GPIOB
#define yB14_Pin GPIO_PIN_14
#define yB14_GPIO_Port GPIOB
#define yB15_Pin GPIO_PIN_15
#define yB15_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_6
#define A_GPIO_Port GPIOC
#define AC7_Pin GPIO_PIN_7
#define AC7_GPIO_Port GPIOC
#define AC12_Pin GPIO_PIN_12
#define AC12_GPIO_Port GPIOC
#define AD2_Pin GPIO_PIN_2
#define AD2_GPIO_Port GPIOD
#define board_LEDB5_Pin GPIO_PIN_5
#define board_LEDB5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
