/*
 * DS18B20.h
 *
 *  Created on: Mar 20, 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_
#include "one_wire.h"
#include "main.h"
#include "stm32f4xx_hal.h"


#define 		SEARCH_ROM 			0xF0
#define  		READ_ROM 			0x33
#define  		SKIP_ROM 			0xCC
#define  		MATCH_ROM 			0x55
#define  		CONVERT 			0x44
#define  		READ_SCRATCHPAD 	0xBE
#define  		WRITE_SCRATCHPAD 	0x4E
#define  		COPY_SCRATCHPAD 	0x48
#define 		SCRATCHPAD_SIZE		9

typedef enum
{
	DS18B20_OK,
	DS18B20_ERROR
}DS18B20_Status_t;

typedef enum{
	DS18B20_9bit_resolution = 9,
	DS18B20_10bit_resolution = 10,
	DS18B20_11bit_resolution = 11,
	DS18B20_12bit_resolution = 12,

}DS18B20_Resolution_t;

typedef enum{
	DS18B20_9bit_conversion_time = 94,
	DS18B20_10bit_conversion_time = 188,
	DS18B20_11bit_conversion_time = 375,
	DS18B20_12bit_conversion_time = 750,

}DS18B20_conversion_time_t;

DS18B20_Status_t DS18B20Init();

uint8_t DS18B20ByteCrc(uint8_t crc, uint8_t byte);

uint8_t DS18B20WireCrc(const uint8_t* data, int len);

DS18B20_Status_t DS18B20ReadSratchpad(uint8_t scratchpad[]);

DS18B20_Status_t DS18B20SetResolution(DS18B20_Resolution_t resolution);

DS18B20_Status_t DS18B20GetTemp(float *temp);

#endif /* INC_DS18B20_H_ */
