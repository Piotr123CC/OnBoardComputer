/*
 * ds18b20.h
 *
 *  Created on: Mar 20, 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_ONE_WIRE_H_
#define INC_ONE_WIRE_H_

#define  GPIO_CONTROL 		0
#define  USART_CONTROL 		1

#include "main.h"

#if USART_CONTROL
#include "usart.h"

#define  DS18B20_UART	huart1	//change to your UART
#define  DS18B20_USART	USART1
#endif

#if GPIO_CONTROL
#include "gpio.h"
#include "tim.h"

#define TIMER 			htim10 //change to your timer
#define DS_GPIO_Port 	GPIOA //change to your GPIO port
#define DS_Pin			GPIO_PIN_9	//change to you GPIO number
#endif

HAL_StatusTypeDef wireReset();
void writeBit(int value);
int readBit(void);
void wireWrite(uint8_t byte);
uint8_t wireRead(void);


#if GPIO_CONTROL
void delay_us(TIM_HandleTypeDef* timer, uint32_t us);
#endif

#if USART_CONTROL
void setBaudrate(uint32_t baudrate);
#endif



#endif /* INC_ONE_WIRE_H_ */
