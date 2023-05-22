/*
 * DS18B20.c
 *
 *  Created on: Mar 20, 2023
 *      Author: Piotr Cytryński
 */


#include "DS18B20.h"
#include "mainProgram.h"
extern mainData_t mainData;


DS18B20_conversion_time_t conversion_time  = DS18B20_12bit_conversion_time;

DS18B20_Status_t DS18B20Init()
{
	wireReset();
	wireWrite(READ_ROM);
	uint8_t romCode[8];
	for (int i =0;i<8;i++)
	{
	  romCode[i]=wireRead();
	}

	if (romCode[0] != 0x29)
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.TMP2 = true;
		mainData.Errors.TMP2Errors++;
		return DS18B20_ERROR;
	}
	else
	{
		mainData.Errors.TMP2 = false;
		return DS18B20_OK;
	}
}


uint8_t DS18B20ByteCrc(uint8_t crc, uint8_t byte)
{
	for (int i=0;i<8;i++)
	{
		uint8_t b = crc ^ byte;
		crc >>= 1;
		if(b & 0x01)
		{
			crc ^= 0x8c;
		}
		byte >>= 1;
	}
	return crc;
}


uint8_t DS18B20WireCrc(const uint8_t* data, int len)
{
	uint8_t crc  =0;
	for(int i=0;i<len;i++)
	{
		crc = DS18B20ByteCrc(crc, data[i]);
	}
	return crc;
}


DS18B20_Status_t DS18B20ReadSratchpad(uint8_t scratchpad[])
{
	wireReset();
	wireWrite(SKIP_ROM);
	wireWrite(READ_SCRATCHPAD);

	for (int i =0;i<SCRATCHPAD_SIZE;i++)
	{
		scratchpad[i]=wireRead();
	}

	uint8_t crc = DS18B20WireCrc(scratchpad,8);

	if (crc != scratchpad[8])
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.TMP2 = true;
		mainData.Errors.TMP2Errors++;
		return DS18B20_ERROR;
	}
	else
	{
		mainData.Errors.TMP2 = false;
		return DS18B20_OK;
	}
}


DS18B20_Status_t DS18B20GetTemp(float *temp)
{
	wireReset();
	wireWrite(SKIP_ROM);
	wireWrite(CONVERT);

	HAL_Delay(conversion_time);

	wireReset();
	wireWrite(SKIP_ROM);
	wireWrite(READ_SCRATCHPAD);

	uint8_t scratchpad[9];
	for (uint8_t i =0;i<SCRATCHPAD_SIZE;i++)
	{
		scratchpad[i]=wireRead();
	}

	uint8_t crc = DS18B20WireCrc(scratchpad, 8);
	if (crc != scratchpad[8] || crc == 0)
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.TMP2 = true;
		mainData.Errors.TMP2Errors++;
		return DS18B20_ERROR;
	}


	*temp = ((scratchpad[1]<<8) | scratchpad[0])/16.0f;

	if (*temp >80 || *temp < -60)
	{
		blinkRedLEDAlarm();
		mainData.Errors.allErrors++;
		mainData.Errors.TMP2 = true;
		mainData.Errors.TMP2Errors++;
		return DS18B20_ERROR;
	}
	mainData.Errors.TMP2 = false;
	return DS18B20_OK;
}


DS18B20_Status_t DS18B20SetResolution(DS18B20_Resolution_t resolution)
{

	uint8_t th, tl, conf, tmpconf;

	wireReset();
	wireWrite(SKIP_ROM);
	wireWrite(READ_SCRATCHPAD);


//	 in order to read configuration register need to read previous 4 bytes
	wireRead();	// temp. LSB
	wireRead();	// temp  MSB
	th = wireRead();	// th reg
	tl = wireRead();	// tl reg
	conf=wireRead();	// conf reg
	tmpconf = conf;
	switch(resolution)
	{
		case DS18B20_9bit_resolution:
			conf &= ~(0x01 << 6);
			conf &= ~(0x01 << 5);
			conversion_time = DS18B20_9bit_conversion_time;
		break;
		case DS18B20_10bit_resolution:
			conf &= ~(0x01 << 6);
			conf |= (0x01 << 5);
			conversion_time = DS18B20_10bit_conversion_time;
		break;
		case DS18B20_11bit_resolution:
			conf |= (0x01 << 6);
			conf &= ~(0x01 << 5);
			conversion_time = DS18B20_11bit_conversion_time;
		break;
		case DS18B20_12bit_resolution:
			conf |= (0x01 << 6);
			conf |= (0x01 << 5);
			conversion_time = DS18B20_12bit_conversion_time;
		break;
	}
	 wireReset();
	 wireWrite(SKIP_ROM);
	 wireWrite(WRITE_SCRATCHPAD);
	 wireWrite(th);
	 wireWrite(tl);
	 wireWrite(conf);

	 wireReset();
	 wireWrite(SKIP_ROM);
	 wireWrite(READ_SCRATCHPAD);
	 wireRead();
	 wireRead();
	 wireRead();
	 wireRead();
	 conf = wireRead();

	 if (conf == tmpconf)
	 {
		 return DS18B20_ERROR;
	 }
	 wireReset();
	 wireWrite(SKIP_ROM);
	 wireWrite(COPY_SCRATCHPAD);

	 return DS18B20_OK;

}
