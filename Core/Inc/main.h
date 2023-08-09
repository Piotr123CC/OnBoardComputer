/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BATTERY_IN_VOLTAGE_Pin GPIO_PIN_0
#define BATTERY_IN_VOLTAGE_GPIO_Port GPIOC
#define BUZZER_PIN_Pin GPIO_PIN_2
#define BUZZER_PIN_GPIO_Port GPIOC
#define BLUE_LED_Pin GPIO_PIN_3
#define BLUE_LED_GPIO_Port GPIOC
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOA
#define NTC_VOLTAGE_Pin GPIO_PIN_1
#define NTC_VOLTAGE_GPIO_Port GPIOA
#define AIR_CONDITIONING_Pin GPIO_PIN_4
#define AIR_CONDITIONING_GPIO_Port GPIOA
#define HCSR04_2_TRIGGER_RF_Pin GPIO_PIN_5
#define HCSR04_2_TRIGGER_RF_GPIO_Port GPIOA
#define HCSR04_3_ECHO_LB_Pin GPIO_PIN_6
#define HCSR04_3_ECHO_LB_GPIO_Port GPIOA
#define YELLOW_LED_Pin GPIO_PIN_4
#define YELLOW_LED_GPIO_Port GPIOC
#define USER_BUTTON_PIN_Pin GPIO_PIN_5
#define USER_BUTTON_PIN_GPIO_Port GPIOC
#define USER_BUTTON_PIN_EXTI_IRQn EXTI9_5_IRQn
#define BATTERY_CAR_VOLTAGE_Pin GPIO_PIN_0
#define BATTERY_CAR_VOLTAGE_GPIO_Port GPIOB
#define HCSR04_3_TRIGGER_LB_Pin GPIO_PIN_1
#define HCSR04_3_TRIGGER_LB_GPIO_Port GPIOB
#define HCSR04_2_ECHO_RF_Pin GPIO_PIN_10
#define HCSR04_2_ECHO_RF_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_9
#define RED_LED_GPIO_Port GPIOC
#define HCSR04_1_ECHO_LF_Pin GPIO_PIN_8
#define HCSR04_1_ECHO_LF_GPIO_Port GPIOA
#define HCSR04_1_TRIGGER_LF_Pin GPIO_PIN_10
#define HCSR04_1_TRIGGER_LF_GPIO_Port GPIOA
#define DISTANCE_SENSORS_BUTTON_Pin GPIO_PIN_11
#define DISTANCE_SENSORS_BUTTON_GPIO_Port GPIOC
#define DISTANCE_SENSORS_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define HCDR04_4_TRIGGER_RB_Pin GPIO_PIN_7
#define HCDR04_4_TRIGGER_RB_GPIO_Port GPIOB
#define HCSR04_4_ECHO_RB_Pin GPIO_PIN_8
#define HCSR04_4_ECHO_RB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
