/*
 * mainProgram.h
 *
 *  Created on: Mar 17, 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_MAINPROGRAM_H_
#define INC_MAINPROGRAM_H_
#include "main.h"
#include "stdbool.h"
typedef struct{

	bool disanceSensorsState;
	bool airConditioningState;
	bool pidState;
	bool ledState;
	bool manualServoControlState;
	bool servoState;


	uint8_t servoPosition;

	struct {
		float leftFront;
		float rightFront;
		float leftBack;
		float rightBack;
	}Distance;

	struct {
		float firstSensor;
		float secondSensor;
		float avgTemperature;
	}Temperature;

	struct {
		float batteryCarVoltage;
		float batteryInVoltage;
		float airConditioningVoltage;
	}Voltage;

	struct Errors{
		uint32_t allErrors;
		uint32_t LFDErrors;
		uint32_t RFDErrors;
		uint32_t LBDErrors;
		uint32_t RBDErrors;
		uint32_t TMP1Errors;
		uint32_t TMP2Errors;
		uint32_t LCDErrors;
		bool LFD;
		bool RFD;
		bool LBD;
		bool RBD;
		bool TMP1;
		bool TMP2;
		bool LCD;
		bool alarmState;
	}Errors;
}mainData_t;

typedef enum {
	Errors,
	numberErrors,
	batteryVoltage,
	insideTemperature,
	outsideTemperature,
	frontDistance,
	backDistance,
	setTemperature,
	manualTemperatureSetting,
}LCDdisplay_t;

#define V_CAR_BATT_CONST 		(14.57f/4096.f)
#define V_AIR_CON_CONST 		(16.0f/4096.f)
#define AIR_CON_OFFSET			13.0f
#define V_IN_BATT_CONST			(8.2f/4096.0f)
#define V_SERVO_POS 			(185.0f/4096.f)
#define A 						0.001111f
#define B 						0.000237987f
#define C 						0.000000065f
#define NTC_R 					10000.0f
#define NTCsensor1				1
#define NTCsensor2				2

#define HCSR04_1_TIM_TRIGGER 			htim1
#define HCSR04_1_TIM_ECHO 				htim1
#define HCSR04_1_TRIGGER_CHANNEL 		TIM_CHANNEL_3
#define HCSR04_1_ECHO_CHANNEL_START 	TIM_CHANNEL_1
#define HCSR04_1_ECHO_CHANNEL_STOP		TIM_CHANNEL_2


#define HCSR04_2_TIM_TRIGGER 			htim2
#define HCSR04_2_TIM_ECHO 				htim2
#define HCSR04_2_TRIGGER_CHANNEL 		TIM_CHANNEL_1
#define HCSR04_2_ECHO_CHANNEL_START 	TIM_CHANNEL_3
#define HCSR04_2_ECHO_CHANNEL_STOP		TIM_CHANNEL_4


#define HCSR04_3_TIM_TRIGGER 			htim3
#define HCSR04_3_TIM_ECHO 				htim3
#define HCSR04_3_TRIGGER_CHANNEL 		TIM_CHANNEL_4
#define HCSR04_3_ECHO_CHANNEL_START 	TIM_CHANNEL_1
#define HCSR04_3_ECHO_CHANNEL_STOP		TIM_CHANNEL_2

#define HCSR04_4_TIM_TRIGGER 			htim4
#define HCSR04_4_TIM_ECHO 				htim4
#define HCSR04_4_TRIGGER_CHANNEL 		TIM_CHANNEL_2
#define HCSR04_4_ECHO_CHANNEL_START 	TIM_CHANNEL_3
#define HCSR04_4_ECHO_CHANNEL_STOP		TIM_CHANNEL_4

#define SG90_TIMER						htim5
#define SG90_PWM_CHANNEL				TIM_CHANNEL_1

#define LED_TIMER						htim11

#define BUZZER_FREQUENCY_1				500
#define BUZZER_FREQUENCY_2				350
#define BUZZER_FREQUENCY_3				150
#define BUZZER_FREQUENCY_4				50

void MainProgram();
void modulesInitialization();
void LCD_init();
void displayBatteryVoltage();
void displayInsideTemperature();
void displayOutsideTemperature();
void displayFrontDistance();
void displayBackDistance();
void displayErrors();
void displayNumberErrors();
void blinkLEDS();
void initBlinkLEDS();
void blinkRedLEDAlarm();
void turnOffLEDS();
void getTempNTC(float *TEMP, uint8_t SensorTEMP1);
void turnOffDistanceSensors();
void turnOnDistanceSensors();
void displaySetTemperature();
void displayManualTemperatureSetting();
void getBatteryInVoltage(float *vBatIn);
void getCarBatteryVoltage(float *vCarBat);
void setBuzzer();
void setServo();
void calculatePID();
void calculateAvgTemp();
float map(float x, float dataInMin, float dataInMax, float dataOutMin, float dataOutMax);
#endif /* INC_MAINPROGRAM_H_ */
