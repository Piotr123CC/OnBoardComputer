/*
 * lcdI2C.c
 *
 *  Created on: 27 lut 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include <mainProgram.h>
extern mainData_t mainData;


#define HI2C_DEF hi2c1



#define RS_PIN 0x01
#define RW_PIN 0x02
#define EN_PIN 0x04
#define BL_PIN 0x08

#define INIT_8_BIT_MODE	0x30
#define INIT_4_BIT_MODE	0x02

#define CLEAR_LCD	0x01

#define UNDERLINE_OFF_BLINK_OFF		0x0C
#define UNDERLINE_OFF_BLINK_ON		0x0D
#define UNDERLINE_ON_BLINK_OFF		0x0E
#define UNDERLINE_ON_BLINK_ON		0x0F

#define FIRST_CHAR_LINE_1	0x80
#define FIRST_CHAR_LINE_2	0xC0

struct lcdDisp {
	uint8_t addr;
	char f_line[17];
	char s_line[17];
	bool bl;
};

void lcdInit(struct lcdDisp * lcd);
void lcdWrite(uint8_t addr, uint8_t data, uint8_t xpin);
void lcdDisplay(struct lcdDisp * lcd);
void lcdClear(struct lcdDisp * lcd);

#endif /* INC_LCD_I2C_H_ */
