/*
 * PID.h
 *
 *  Created on: Jan 11, 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "stdint.h"

typedef struct
{
	//Controller gains
	float Kp;
	float Ki;
	float Kd;



	//output limits
	float limMin;
	float limMax;

	//Controller "Memory"
	float prevError;
	float totalError;

	uint16_t out;

}PIDController;


void PIDControllerInit(PIDController *pid);
float PIDControllerUpdate(PIDController *pid, float setpoint, float measurement);


#endif /* INC_PID_H_ */


