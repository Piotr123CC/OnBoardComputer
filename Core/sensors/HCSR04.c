/*
 * HCSR04.c
 *
 *  Created on: 27 lut 2023
 *      Author: Piotr Cytryński
 */

#include "main.h"
#include "HCSR04.h"
#include "gpio.h"
#include "tim.h"


HCSR04_Status_t HCSR04_CalculateResultFloat(HCSR04_t *hcsr04, float *Result)
{
	*Result = HCSR04_CALCULATION_CONST * (float)hcsr04->Result_us;

	if (*Result > 1.0f && *Result < 400.0f)
	{
		return HCSR04_OK;
	}
	else
	{
		return HCSR04_ERROR;

	}

}

HCSR04_Status_t HCSR04_CalculateResultInteger(HCSR04_t *hcsr04, uint16_t *Result)
{
	*Result = (float)hcsr04->Result_us/58;

	return HCSR04_OK;
}
void HCSR04_interrupt_handler(HCSR04_t *hcsr04)
{
	hcsr04->Result_us = __HAL_TIM_GET_COMPARE(hcsr04->htim_echo, hcsr04->Echo_tim_channel_stop) -
						__HAL_TIM_GET_COMPARE(hcsr04->htim_echo, hcsr04->Echo_tim_channel_start);
}

HCSR04_Status_t HCSR04_Init(HCSR04_t *hcsr04,
		TIM_HandleTypeDef *timer_trigger,
		uint32_t Trigger_tim_channel,
		TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_tim_channel_start,
		uint32_t Echo_tim_channel_stop)
{
	hcsr04->htim_tirgger = timer_trigger;
	hcsr04->Trigger_tim_channel =  Trigger_tim_channel;

	hcsr04->htim_echo =timer_echo;

	hcsr04->Echo_tim_channel_start =  Echo_tim_channel_start;
	hcsr04->Echo_tim_channel_stop =  Echo_tim_channel_stop;

	HAL_TIM_Base_Start(hcsr04->htim_echo);

	HAL_TIM_Base_Start(hcsr04->htim_tirgger);
	HAL_TIM_IC_Start(hcsr04->htim_echo, hcsr04->Echo_tim_channel_start);
	HAL_TIM_IC_Start_IT(hcsr04->htim_echo, hcsr04->Echo_tim_channel_stop);

	HAL_TIM_PWM_Start(hcsr04->htim_tirgger, hcsr04->Trigger_tim_channel);
	return HCSR04_OK;
}


HCSR04_Status_t HCSR04_DeInit(HCSR04_t *hcsr04,
		TIM_HandleTypeDef *timer_trigger,
		uint32_t Trigger_tim_channel,
		TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_tim_channel_start,
		uint32_t Echo_tim_channel_stop)
{
	hcsr04->htim_tirgger = timer_trigger;
	hcsr04->Trigger_tim_channel =  Trigger_tim_channel;

	hcsr04->htim_echo =timer_echo;

	hcsr04->Echo_tim_channel_start =  Echo_tim_channel_start;
	hcsr04->Echo_tim_channel_stop =  Echo_tim_channel_stop;

	HAL_TIM_Base_Stop(hcsr04->htim_echo);

	HAL_TIM_Base_Stop(hcsr04->htim_tirgger);
	HAL_TIM_IC_Stop(hcsr04->htim_echo, hcsr04->Echo_tim_channel_start);
	HAL_TIM_IC_Stop_IT(hcsr04->htim_echo, hcsr04->Echo_tim_channel_stop);

	HAL_TIM_PWM_Stop(hcsr04->htim_tirgger, hcsr04->Trigger_tim_channel);
	return HCSR04_OK;
}
