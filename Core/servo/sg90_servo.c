/*
 * sg90_servo.c
 *
 *  Created on: 9 kwi 2023
 *      Author: Piotr Cytryński
 */


#include "sg90_servo.h"


sg90_servo_status_t servoInit(sg90_servo_t *sg90, TIM_HandleTypeDef *servoTimer, uint32_t servoCHannel)
{

	sg90->timer = servoTimer;

	sg90->channel = servoCHannel;

	HAL_TIM_Base_Start(sg90->timer);

	HAL_TIM_PWM_Start(sg90->timer,sg90->channel);

	return SG90_OK;
}

sg90_servo_status_t servoDeInit(sg90_servo_t *sg90, TIM_HandleTypeDef *servoTimer, uint32_t servoCHannel)
{
	sg90->timer = servoTimer;

	sg90->channel = servoCHannel;

	HAL_TIM_Base_Stop(sg90->timer);

	HAL_TIM_PWM_Stop(sg90->timer,sg90->channel);

	return SG90_OK;
}

void servoSetAngle(sg90_servo_t *servo, uint8_t angle)
{
	if (angle > MAX_ANGLE)
	{
		angle = MAX_ANGLE;
	}
	else if (angle < MIN_ANGLE)
	{
		angle = MIN_ANGLE;
	}
	else if (angle < 2)
	{
		HAL_TIM_PWM_Stop(servo->timer, servo->channel);
	}

	uint16_t pos = 0;

	pos = MIN_PWM + (angle * STEP)/1000;

	__HAL_TIM_SET_COMPARE(servo->timer, servo->channel, pos);
}
