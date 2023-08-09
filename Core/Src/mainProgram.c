/*
 * mainProgram.c
 *
 *  Created on: Mar 17, 2023
 *      Author: Piotr Cytryński
 */
#include "main.h"
#include "stdio.h"
#include <lcd_i2c.h>
#include "mainProgram.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "math.h"
#include "one_wire.h"
#include <DS18B20.h>
#include <HCSR04.h>
#include <sg90_servo.h>
#include <PID.h>
struct lcdDisp disp;

extern mainData_t mainData;

extern HCSR04_t hcsr04_1_LF;
extern HCSR04_t hcsr04_2_RF;
extern HCSR04_t hcsr04_3_LB;
extern HCSR04_t hcsr04_4_RB;

extern sg90_servo_t servo;

extern PIDController pid;
uint16_t adcRaw[10][4];

float tempOut1 =14.57;

extern uint8_t displayInfo;

uint32_t mainProgramTimer = 0;
extern uint16_t buzzerTime;
uint32_t pidTimer = 0;
volatile uint8_t ledCounter = 0;



void MainProgram()
{


	if ((HAL_GetTick() - mainProgramTimer) > 700)
	{

	switch (displayInfo) {
		case Errors:	//errors
			displayErrors();
			break;
		case numberErrors: //Number errors
			displayNumberErrors();
			break;
		case batteryVoltage:	//Battery voltage
			displayBatteryVoltage();
			break;
		case insideTemperature:	// temperature inside
			displayInsideTemperature();
			break;
		case outsideTemperature:	// temperature outside
			displayOutsideTemperature();
			break;
		case frontDistance: // distance forwad
			displayFrontDistance();
			break;
		case backDistance:	// distance backward
			displayBackDistance();
			break;
		case setTemperature:	// automatic temperature control
			displaySetTemperature();
			break;
		case manualTemperatureSetting:	// manual temperature control
			displayManualTemperatureSetting();
			break;
		default:
			break;
	}

	mainProgramTimer = HAL_GetTick();
	}

	setBuzzer();

	setServo();

	getBatteryInVoltage(&mainData.Voltage.batteryInVoltage);
	getCarBatteryVoltage(&mainData.Voltage.batteryCarVoltage);

	calculatePID();

	getTempNTC(&mainData.Temperature.firstSensor, NTCsensor1);

	DS18B20GetTemp(&mainData.Temperature.secondSensor);

	calculateAvgTemp();
}


void modulesInitialization()
{
	  disp.addr = (0x27<<1);
	  disp.bl = true;
	  lcdInit(&disp);
	  LCD_init();
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRaw, 40);
	  DS18B20Init();
	  DS18B20SetResolution(DS18B20_10bit_resolution);
	  uint8_t schratchPad[9];
	  DS18B20ReadSratchpad(schratchPad);
	  turnOnDistanceSensors();
	  servoInit(&servo, &SG90_TIMER, SG90_PWM_CHANNEL);
	  mainData.airConditioningState = false;
	  PIDControllerInit(&pid);
}

void displayBatteryVoltage()
{
	sprintf((char *)disp.f_line,"Car bat.=%.2f",mainData.Voltage.batteryCarVoltage);
	sprintf((char *)disp.s_line,"In. bat.=%.2f",mainData.Voltage.batteryInVoltage);
	lcdDisplay(&disp);
}


void displayInsideTemperature()
{
	sprintf((char *)disp.f_line,"Temp1.in.=%.2f",mainData.Temperature.firstSensor);
	sprintf((char *)disp.s_line,"Temp2.in.=%.2lf",mainData.Temperature.secondSensor);
	lcdDisplay(&disp);

}
void displayOutsideTemperature()
{

	// TODO
	sprintf((char *)disp.f_line,"Temp.out.=%.2f",tempOut1);
	sprintf((char *)disp.s_line,"hum.out.=70");
	lcdDisplay(&disp);
}
void displayFrontDistance()
{
	if(mainData.disanceSensorsState == true)
	{
		sprintf((char *)disp.f_line,"L.F.dist.=%.2f",mainData.Distance.leftFront);
		sprintf((char *)disp.s_line,"R.F.dist.=%.2f",mainData.Distance.rightFront);
	}
	else
	{
		sprintf((char *)disp.f_line,"Front Sensors");
		sprintf((char *)disp.s_line," 	 OFF ");
	}

	lcdDisplay(&disp);
}
void displayBackDistance()
{

	if(mainData.disanceSensorsState == true)
	{
		sprintf((char *)disp.f_line,"L.B.dist.=%.2f",mainData.Distance.leftBack);
		sprintf((char *)disp.s_line,"R.B.dist.=%.2f",mainData.Distance.rightBack);
	}
	else
	{
		sprintf((char *)disp.f_line,"Back Sensors");
		sprintf((char *)disp.s_line," 	 OFF ");
	}

	lcdDisplay(&disp);

}

void displayNumberErrors()
{
	sprintf((char *)disp.f_line,"ALL ERRORS");
	sprintf((char *)disp.s_line,"      %lu",mainData.Errors.allErrors);
	lcdDisplay(&disp);
}

void displayErrors()
{
	static uint8_t errorNumber = 0;
	switch(errorNumber)
	{
	case 0:
		errorNumber++;
		sprintf((char *)disp.f_line,"L.F.dist = %lu",mainData.Errors.LFDErrors);
		sprintf((char *)disp.s_line,"R.F.dist = %lu",mainData.Errors.RFDErrors);
		lcdDisplay(&disp);
		break;
	case 1:
		errorNumber++;
		sprintf((char *)disp.f_line,"L.B.dist = %lu",mainData.Errors.LBDErrors);
		sprintf((char *)disp.s_line,"R.B.dist = %lu",mainData.Errors.RBDErrors);
		lcdDisplay(&disp);
		break;
	case 2:
		errorNumber++;
		sprintf((char *)disp.f_line,"Temp1 = %lu",mainData.Errors.TMP1Errors);
		sprintf((char *)disp.s_line,"Temp2 = %lu",mainData.Errors.TMP2Errors);
		lcdDisplay(&disp);
		break;
	case 3:
		errorNumber++;
		sprintf((char *)disp.f_line,"LCD = %lu",mainData.Errors.LBDErrors);
		sprintf((char *)disp.f_line," ");
		lcdDisplay(&disp);
		break;

	}
	if (errorNumber == 3)
	{
		errorNumber = 0;
	}

}


void LCD_init()
{
	mainData.ledState = true;
	HAL_TIM_Base_Start_IT(&LED_TIMER);
	sprintf((char *)disp.f_line, "Computer Init");
	lcdDisplay(&disp);
	sprintf((char *)disp.s_line,"." );
	HAL_Delay(1000);
	lcdDisplay(&disp);
	sprintf((char *)disp.s_line,".." );
	HAL_Delay(1000);
	lcdDisplay(&disp);
	sprintf((char *)disp.s_line,"..." );
	HAL_Delay(1000);
	lcdDisplay(&disp);
	sprintf((char *)disp.s_line,"...." );
	HAL_Delay(1000);
	lcdDisplay(&disp);
	sprintf((char *)disp.s_line,"....." );
	HAL_Delay(1000);
	lcdDisplay(&disp);
	HAL_Delay(1000);
	turnOffLEDS();

	lcdClear(&disp);
}

void getCarBatteryVoltage(float *vCarBat)
{
	uint16_t Vbat_avg =0;
	for (int i = 0;i<10;i++)
	{
		Vbat_avg += adcRaw[i][0];
	}
	Vbat_avg /= 10;
	*vCarBat = Vbat_avg * V_CAR_BATT_CONST;

}

void getBatteryInVoltage(float *vBatIn)
{
	uint16_t Vbat_avg =0;
	for (int i = 0;i<10;i++)
	{
		Vbat_avg += adcRaw[i][3];
	}
	Vbat_avg /= 10;
	*vBatIn = Vbat_avg * V_IN_BATT_CONST;
	if (*vBatIn < 6.4f)
	{
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
	}
}
void blinkLEDS()
{
	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
	HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);

}

void turnOffLEDS()
{
	mainData.ledState = false;
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
}

void blinkRedLEDAlarm()
{
	static uint8_t counter = 0;
	counter++;
	if (counter > 4)
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	}
	if (counter > 8)
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		counter =0;
		mainData.Errors.alarmState = false;
	}
}
void initBlinkLEDS()
{
	static volatile uint8_t ledCounter = 0;

	if ( (ledCounter==0) || (ledCounter==1))
	{
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	}

	else if ( (ledCounter==2) || (ledCounter==3))
	{
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	}
	else if ( (ledCounter==4) || (ledCounter==5))
	{
		HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);
	}
	else if ( (ledCounter==6) || (ledCounter==7))
	{
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
	}
	ledCounter++;
	if (ledCounter == 8)
	{
		ledCounter = 0;
	}
}



void getTempNTC(float *TEMP, uint8_t SensorTEMP1)
{
	float temp1Mean =0;

	for (uint8_t i =0;i<10;i++)
	{
		temp1Mean += adcRaw[i][SensorTEMP1];
	}

	double tmpTEMP1 =  temp1Mean/10;

	double NTC_R1 = ((NTC_R)/((4095.0f/tmpTEMP1)- 1.0f));

	*TEMP =	log(NTC_R1);

	*TEMP = (1.0/(A + B *(*TEMP) + C *(*TEMP)*(*TEMP)*(*TEMP))) - 273.15;

	if (*TEMP > -40.0f && *TEMP <= 100.0f)
	{
		mainData.Errors.TMP1 = false;
	}
	else
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.TMP1 = true;
		mainData.Errors.TMP1Errors++;
	}
}


void turnOnDistanceSensors()
{
	mainData.disanceSensorsState = true;
	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
	HCSR04_Init(&hcsr04_1_LF,
		  &HCSR04_1_TIM_TRIGGER,
		  HCSR04_1_TRIGGER_CHANNEL,
		  &HCSR04_1_TIM_ECHO,
		  HCSR04_1_ECHO_CHANNEL_START,
		  HCSR04_1_ECHO_CHANNEL_STOP);

	HCSR04_Init(&hcsr04_2_RF,
		  &HCSR04_2_TIM_TRIGGER,
		  HCSR04_2_TRIGGER_CHANNEL,
		  &HCSR04_2_TIM_ECHO,
		  HCSR04_2_ECHO_CHANNEL_START,
		  HCSR04_2_ECHO_CHANNEL_STOP);

	HCSR04_Init(&hcsr04_3_LB,
		  &HCSR04_3_TIM_TRIGGER,
		  HCSR04_3_TRIGGER_CHANNEL,
		  &HCSR04_3_TIM_ECHO,
		  HCSR04_3_ECHO_CHANNEL_START,
		  HCSR04_3_ECHO_CHANNEL_STOP);

	HCSR04_Init(&hcsr04_4_RB,
		  &HCSR04_4_TIM_TRIGGER,
		  HCSR04_4_TRIGGER_CHANNEL,
		  &HCSR04_4_TIM_ECHO,
		  HCSR04_4_ECHO_CHANNEL_START,
		  HCSR04_4_ECHO_CHANNEL_STOP);
 }

void turnOffDistanceSensors()
{
	mainData.disanceSensorsState = false;
	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
	HCSR04_DeInit(&hcsr04_1_LF,
		  &HCSR04_1_TIM_TRIGGER,
		  HCSR04_1_TRIGGER_CHANNEL,
		  &HCSR04_1_TIM_ECHO,
		  HCSR04_1_ECHO_CHANNEL_START,
		  HCSR04_1_ECHO_CHANNEL_STOP);

	HCSR04_DeInit(&hcsr04_2_RF,
		  &HCSR04_2_TIM_TRIGGER,
		  HCSR04_2_TRIGGER_CHANNEL,
		  &HCSR04_2_TIM_ECHO,
		  HCSR04_2_ECHO_CHANNEL_START,
		  HCSR04_2_ECHO_CHANNEL_STOP);

	HCSR04_DeInit(&hcsr04_3_LB,
		  &HCSR04_3_TIM_TRIGGER,
		  HCSR04_3_TRIGGER_CHANNEL,
		  &HCSR04_3_TIM_ECHO,
		  HCSR04_3_ECHO_CHANNEL_START,
		  HCSR04_3_ECHO_CHANNEL_STOP);

	HCSR04_DeInit(&hcsr04_4_RB,
		  &HCSR04_4_TIM_TRIGGER,
		  HCSR04_4_TRIGGER_CHANNEL,
		  &HCSR04_4_TIM_ECHO,
		  HCSR04_4_ECHO_CHANNEL_START,
		  HCSR04_4_ECHO_CHANNEL_STOP);
}


void displaySetTemperature()
{
	uint16_t avgSetTemp = 0 ;
	for (uint8_t i=0;i<10;i++)
	{
		avgSetTemp += adcRaw[i][2];
	}
	avgSetTemp /= 10;

	mainData.Voltage.airConditioningVoltage = avgSetTemp * V_AIR_CON_CONST + AIR_CON_OFFSET;

	if (mainData.Voltage.airConditioningVoltage > 16)
	{
		if (mainData.servoState == false)
		{
			servoInit(&servo, &htim5, TIM_CHANNEL_1);
		}
		mainData.pidState = true;
		mainData.manualServoControlState = false;
		mainData.airConditioningState = true;
		HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);

		sprintf((char *)disp.f_line,"Set Temp.= %.2f", mainData.Voltage.airConditioningVoltage);
		sprintf((char *)disp.s_line,"In. Temp.= %.2f",mainData.Temperature.avgTemperature);
	}
	else
	{
		mainData.pidState = false;
		mainData.airConditioningState = false;
		HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET);
		sprintf((char *)disp.f_line,"Air Conditioning");
		sprintf((char *)disp.s_line,"     OFF");
	}

	lcdDisplay(&disp);
}


void displayManualTemperatureSetting()
{
	if (mainData.airConditioningState == false)
	{
		mainData.manualServoControlState = true;
		sprintf((char *)disp.f_line,"Man.temp.control");
		sprintf((char *)disp.f_line,"Servo pos. = %d",mainData.servoPosition);
		if (mainData.servoPosition < 2  && mainData.manualServoControlState == true)
		{
			servoDeInit(&servo, &htim5, TIM_CHANNEL_1);
			sprintf((char *)disp.f_line,"Servo OFF");
			sprintf((char *)disp.s_line," ");
			mainData.manualServoControlState = false;
			mainData.servoState = false;
			if (adcRaw[0][2] > 80)
			{
				mainData.manualServoControlState = true;
			}
		}

		lcdDisplay(&disp);
	}

	else
	{
		mainData.manualServoControlState = false;
		sprintf((char *)disp.f_line,"Turn off AC");
		sprintf((char *)disp.s_line," ");
		lcdDisplay(&disp);
	}

	if (mainData.manualServoControlState == true && mainData.servoPosition >= 2)
	{
		servoInit(&servo, &SG90_TIMER, SG90_PWM_CHANNEL);
		mainData.servoState = true;
	}
}


void setBuzzer()
{
	if (mainData.disanceSensorsState == true)
	{

		if ((mainData.Distance.leftFront >1 && mainData.Distance.leftFront < 20) ||
				 (mainData.Distance.rightFront >1 && mainData.Distance.rightFront < 20) ||
				 (mainData.Distance.leftBack >1 && mainData.Distance.leftBack < 20) ||
				 (mainData.Distance.rightBack >1 && mainData.Distance.rightBack < 20))
		{
			buzzerTime = BUZZER_FREQUENCY_4;
		}
		else if ((mainData.Distance.leftFront >=20 && mainData.Distance.leftFront < 30) ||
				 (mainData.Distance.rightFront >=20 && mainData.Distance.rightFront < 30) ||
				 (mainData.Distance.leftBack >=20 && mainData.Distance.leftBack < 30) ||
				 (mainData.Distance.rightBack >=20 && mainData.Distance.rightBack < 30))
		{
			buzzerTime = BUZZER_FREQUENCY_3;
		}
		else if ((mainData.Distance.leftFront >=30 && mainData.Distance.leftFront < 40) ||
				 (mainData.Distance.rightFront >=30 && mainData.Distance.rightFront < 40) ||
				 (mainData.Distance.leftBack >=30 && mainData.Distance.leftBack < 40) ||
				 (mainData.Distance.rightBack >=30 && mainData.Distance.rightBack < 40))
		{
			buzzerTime = BUZZER_FREQUENCY_2;
		}
		else if ((mainData.Distance.leftFront >=40 && mainData.Distance.leftFront < 50) ||
			(mainData.Distance.rightFront >=40 && mainData.Distance.rightFront < 50) ||
			(mainData.Distance.leftBack >=40 && mainData.Distance.leftBack < 50) ||
			(mainData.Distance.rightBack >=40 && mainData.Distance.rightBack < 50))
		{
			buzzerTime = BUZZER_FREQUENCY_1;
		}
	}
	else
	{
		HAL_GPIO_WritePin(BUZZER_PIN_GPIO_Port, BUZZER_PIN_Pin, GPIO_PIN_RESET);
	}
}


void setServo()
{
	if (mainData.manualServoControlState == true)
	{
		uint16_t avgSetTemp = 0 ;
		for (uint8_t i=0;i<10;i++)
		{
			avgSetTemp += adcRaw[i][2];
		}
		avgSetTemp /= 10;

		mainData.servoPosition = avgSetTemp * V_SERVO_POS;
		servoSetAngle(&servo, mainData.servoPosition);
	}
}

void calculatePID()
{
	if (mainData.pidState == true)
	{
		PIDControllerUpdate(&pid, mainData.Voltage.airConditioningVoltage, mainData.Temperature.avgTemperature);
		if ((HAL_GetTick() - pidTimer) > 5000 )
		{
			servoSetAngle(&servo, pid.out);
			pidTimer = HAL_GetTick();
		}

	}
}

void calculateAvgTemp()
{
	mainData.Temperature.avgTemperature = (mainData.Temperature.firstSensor + mainData.Temperature.secondSensor)/2;
}

float map(float x, float dataInMin, float dataInMax, float dataOutMin, float dataOutMax)
{
	return (x-dataInMin) * (dataOutMax - dataOutMin) / (dataInMax- dataInMin) + dataOutMin;
}
