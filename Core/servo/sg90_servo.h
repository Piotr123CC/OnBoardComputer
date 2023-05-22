/*
 * sg90_servo.h
 *
 *  Created on: 9 kwi 2023
 *      Author: Piotr Cytryński
 */

#include "main.h"
#include "tim.h"

#define MIN_PWM 	500
#define MAX_PWM 	2600
#define MAX_ANGLE   185
#define MIN_ANGLE   2
#define STEP 		((1000*(MAX_PWM - MIN_PWM))/(MAX_ANGLE - MIN_ANGLE))
typedef enum{
	SG90_OK 	= 0,
	SG90_ERROR 	= 1
}sg90_servo_status_t;

typedef struct{
	TIM_HandleTypeDef *timer;
	uint32_t channel;
}sg90_servo_t;

sg90_servo_status_t servoInit(sg90_servo_t *sg90, TIM_HandleTypeDef *servoTimer, uint32_t servoCHannel);
sg90_servo_status_t servoDeInit(sg90_servo_t *sg90, TIM_HandleTypeDef *servoTimer, uint32_t servoCHannel);


void servoSetAngle(sg90_servo_t *servo, uint8_t angle);
