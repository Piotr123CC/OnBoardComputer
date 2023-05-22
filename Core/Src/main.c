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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "mainProgram.h"
#include <HCSR04.h>
#include <sg90_servo.h>
#include <PID.h>
#include <lcd_i2c.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
mainData_t mainData;
LCDdisplay_t displayInfo;
HCSR04_t hcsr04_1;
HCSR04_t hcsr04_2;
HCSR04_t hcsr04_3;
HCSR04_t hcsr04_4;
sg90_servo_t servo;
PIDController pid = {10,5,3,0,180};


volatile uint32_t currentMillis =0;
volatile uint32_t previousMillis =0;

volatile uint32_t previousMillisBuzzer = 0;
volatile uint32_t buzzerTimer = 0;
volatile uint16_t buzzerTime = 0;

uint16_t servoPos = 500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  modulesInitialization();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MainProgram();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM1_TRG_COM_TIM11_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	currentMillis = HAL_GetTick();

	if ((GPIO_Pin == DISTANCE_SENSORS_BUTTON_Pin) && (currentMillis - previousMillis > 360))
	{
		if ( mainData.disanceSensorsState == true)
		{
			turnOffDistanceSensors();
		}
		else
		{
			turnOnDistanceSensors();
		}
		previousMillis = currentMillis;
	}


	if ((GPIO_Pin == USER_BUTTON_PIN_Pin) && (currentMillis - previousMillis > 350) )
	{
		displayInfo++;

		if (displayInfo > 8)
		{
			displayInfo = 0;
		}
		previousMillis = currentMillis;
	}

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == hcsr04_1.htim_echo)
	{
		HCSR04_interrupt_handler(&hcsr04_1);
		if (HCSR04_CalculateResultFloat(&hcsr04_1, &mainData.Distance.leftFront) != HCSR04_OK)
		{
			mainData.Errors.alarmState = true;
			mainData.Errors.allErrors++;
			mainData.Errors.LFD = true;
			mainData.Errors.LFDErrors++;
		}
		else
		{
			mainData.Errors.LFD = false;
		}
	}

	if (htim == hcsr04_2.htim_echo)
	{
		HCSR04_interrupt_handler(&hcsr04_2);
		if (HCSR04_CalculateResultFloat(&hcsr04_2, &mainData.Distance.rightFront)!= HCSR04_OK)
		{
			mainData.Errors.alarmState = true;
			mainData.Errors.allErrors++;
			mainData.Errors.RFD = true;
			mainData.Errors.RFDErrors++;
		}
		else
		{
			mainData.Errors.RFD = false;
		}
	}


	if (htim == hcsr04_3.htim_echo)
	{
		HCSR04_interrupt_handler(&hcsr04_3);
		if (HCSR04_CalculateResultFloat(&hcsr04_3, &mainData.Distance.leftBack)!= HCSR04_OK)
		{
			mainData.Errors.alarmState = true;
			mainData.Errors.allErrors++;
			mainData.Errors.LBD = true;
			mainData.Errors.LBDErrors++;
		}
		else
		{
			mainData.Errors.LBD = false;
		}
	}

	if (htim == hcsr04_4.htim_echo)
	{
		HCSR04_interrupt_handler(&hcsr04_4);
		if (HCSR04_CalculateResultFloat(&hcsr04_4, &mainData.Distance.rightBack)!= HCSR04_OK)
		{
			mainData.Errors.alarmState = true;
			mainData.Errors.allErrors++;
			mainData.Errors.RBD = true;
			mainData.Errors.RBDErrors++;
		}
		else
		{
			mainData.Errors.RBD = false;
		}
	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	uint32_t currentMillsBuzzer = HAL_GetTick();
	if ((htim->Instance == TIM11) && mainData.ledState == true){
		initBlinkLEDS();
	}

	if (htim->Instance == TIM11 && mainData.Errors.alarmState)
	{
		blinkRedLEDAlarm();
	}

	if ((currentMillsBuzzer - previousMillisBuzzer > buzzerTime) &&(
			((mainData.Distance.leftFront < 50) && (mainData.Distance.leftFront > 1))	||
			((mainData.Distance.rightFront < 50) && (mainData.Distance.rightFront > 1)) ||
			((mainData.Distance.leftBack < 50) && (mainData.Distance.leftBack > 1))	||
			((mainData.Distance.rightBack < 50) && (mainData.Distance.rightBack > 1))
		) && mainData.disanceSensorsState == true)

	{
		HAL_GPIO_TogglePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin);
		previousMillisBuzzer = currentMillsBuzzer;
	}
	else
	{
		HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
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
  __disable_irq();
  while (1)
  {
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
