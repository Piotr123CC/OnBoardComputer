/*
 * PID.cpp
 *
 *  Created on: Jan 11, 2023
 *      Author: Piotr Cytryński
 */


#include "PID.h"



void PIDControllerInit(PIDController *pid)
{
	pid->prevError =0;
	pid->totalError = 0;

}


float PIDControllerUpdate(PIDController *pid, float setpoint, float measurement)
{

	float P, I, D;

	float error = setpoint - measurement;

	P = pid->Kp * error;

	pid->totalError += error;

	I = pid->Ki * pid->totalError;

	if (I > pid->limMax)
	{
		I =pid->limMax;
	}
	else if (I < pid->limMin)
	{
		I = pid->limMin;
	}

	D = pid->Kd*(error - pid->prevError);


	pid->prevError = error;

	pid->out = P + I + D;

	if (pid->out > pid->limMax)
	{
		pid->out =pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{
		pid->out = pid->limMin;
	}

	return pid->out;

}

