/*
 * DS18b20.c
 *
 *  Created on: Mar 20, 2023
 *      Author: Piotr Cytryński
 */

#include "one_wire.h"


#if GPIO_CONTROL
void delay_us(TIM_HandleTypeDef* timer, uint32_t us)
{
	timer->Instance->CNT = 0;
	while(timer->Instance->CNT < us);

}
#endif
HAL_StatusTypeDef wireReset()
{
#if USART_CONTROL
	uint8_t dataOut = 0xF0; //SEARCH ROM
	uint8_t dataIn = 0;
	setBaudrate(9600);

	HAL_UART_Transmit(&DS18B20_UART, &dataOut,1,50);
	HAL_UART_Receive(&DS18B20_UART, &dataIn, 1, 50);
	setBaudrate(115200);
#endif
#if GPIO_CONTROL
	int rc;
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	delayUs(&TIMER,480);
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	delayUs(&TIMER,70);
	rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	delayUs(&TIMER,410);

	if (rc == 0)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
#endif

#if USART_CONTROL
	if (dataIn != 0xF0)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
#endif
}

#if USART_CONTROL
void setBaudrate(uint32_t baudrate)
{
	DS18B20_UART.Instance = DS18B20_USART;
	DS18B20_UART.Init.BaudRate = baudrate;
	DS18B20_UART.Init.WordLength = UART_WORDLENGTH_8B;
	DS18B20_UART.Init.StopBits = UART_STOPBITS_1;
	DS18B20_UART.Init.Parity = UART_PARITY_NONE;
	DS18B20_UART.Init.Mode = UART_MODE_TX_RX;
	DS18B20_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	DS18B20_UART.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&DS18B20_UART) != HAL_OK)
	{
	Error_Handler();
	}
}
#endif

void writeBit(int value)
{
#if USART_CONTROL
	if (value)
	{
		uint8_t dataOut = 0xFF;
		HAL_UART_Transmit(&DS18B20_UART, &dataOut, 1, 1000);
	}
	else
	{
		uint8_t dataOut = 0x0;
		HAL_UART_Transmit(&DS18B20_UART, &dataOut, 1, 1000);
	}
#endif
#if GPIO_CONTROL
	if (value)
	{
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		delayUs(&TIMER, 6);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		delayUs(&TIMER, 64);
	}
	else
	{
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		delayUs(&TIMER, 60);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		delayUs(&TIMER, 10);
	}
#endif
}


int readBit(void)
{
#if USART_CONTROL
	uint8_t dataOut = 0xFF;
	uint8_t dataIn = 0;
	HAL_UART_Transmit(&DS18B20_UART, &dataOut, 1, 100);
	HAL_UART_Receive(&DS18B20_UART, &dataIn, 1, 10);

	return dataIn & 0x01;
#endif
#if GPIO_CONTROL
	 int rc;
	 HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	 delayUs(&TIMER, 6);
	 HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	 delayUs(&TIMER, 9);
	 rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	 delayUs(&TIMER, 55);
	 return rc;
#endif
}

void wireWrite(uint8_t byte)
{
	for(int i=0;i<8;i++)
	{
		writeBit(byte & 0x01);
		byte >>=1;
	}
}
uint8_t wireRead(void)
{
	uint8_t value =0;
	for (int i =0;i<8;i++)
	{
		value >>= 1;
		if (readBit())
		{
			value |= 0x80;
		}
	}
	return value;
}

