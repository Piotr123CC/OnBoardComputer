/*
 * lcdI2C.c
 *
 *  Created on: 27 lut 2023
 *      Author: Piotr Cytryński
 */
#include "lcd_i2c.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"



void lcdInit(struct lcdDisp * lcd)
{
	uint8_t xpin = 0;
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}
	//init sequence
	HAL_Delay(40);
	lcdWrite(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(5);
	lcdWrite(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(1);
	lcdWrite(lcd->addr, INIT_8_BIT_MODE, xpin);

	lcdWrite(lcd->addr, INIT_4_BIT_MODE, xpin);


	lcdWrite(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin);


	lcdClear(lcd);

}

void lcdWrite(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t txData[4];


	txData[0] = (data & 0xF0) | EN_PIN | xpin;
	txData[1] = (data & 0xF0) | xpin;
	txData[2] = (data << 4) | EN_PIN | xpin;
	txData[3] = (data << 4) | xpin;

	if (HAL_I2C_Master_Transmit(&HI2C_DEF, addr, txData, 4, 100) != HAL_OK)
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.LCD = true;
		mainData.Errors.LCDErrors++;
	}
	else
	{
		mainData.Errors.LCD = false;
	}

	HAL_Delay(5);
}

void lcdDisplay(struct lcdDisp * lcd)
{
	uint8_t xpin = 0, i = 0;


	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcdClear(lcd);

	//first data line
	lcdWrite(lcd->addr, FIRST_CHAR_LINE_1, xpin);
	while(lcd->f_line[i])
	{
		lcdWrite(lcd->addr, lcd->f_line[i], (xpin | RS_PIN));
		i++;
	}

	i = 0;
	//second data line
	lcdWrite(lcd->addr, FIRST_CHAR_LINE_2, xpin);
	while(lcd->s_line[i])
	{
		lcdWrite(lcd->addr, lcd->s_line[i], (xpin | RS_PIN));
		i++;
	}
}

void lcdClear(struct lcdDisp * lcd)
{
	uint8_t xpin = 0;


	if(lcd->bl)
	{
		xpin = BL_PIN;
	}


	lcdWrite(lcd->addr, CLEAR_LCD, xpin);
}
