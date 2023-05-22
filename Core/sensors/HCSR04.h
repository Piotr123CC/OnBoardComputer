/*
 * HCSR04.h
 *
 *  Created on: 27 lut 2023
 *      Author: Piotr Cytryński
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#define HCSR04_CALCULATION_CONST ((float)0.01715)

typedef enum
{
	HCSR04_OK,
	HCSR04_ERROR
}HCSR04_Status_t;
typedef struct
{
	TIM_HandleTypeDef 		*htim_tirgger;
	TIM_HandleTypeDef 		*htim_echo;

	uint32_t				Trigger_tim_channel;


	uint32_t				Echo_tim_channel_start;
	uint32_t 				Echo_tim_channel_stop;

	uint16_t 				Result_us;

}HCSR04_t;

HCSR04_Status_t HCSR04_CalculateResultFloat(HCSR04_t *hcsr04, float *Result);
HCSR04_Status_t HCSR04_CalculateResultInteger(HCSR04_t *hcsr04, uint16_t *Result);
void HCSR04_interrupt_handler(HCSR04_t *hcsr04);
HCSR04_Status_t HCSR04_Init(HCSR04_t *hcsr04,
		TIM_HandleTypeDef *timer_trigger,
		uint32_t Trigger_tim_channel,
		TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_tim_channel_start,
		uint32_t Echo_tim_channel_stop);


HCSR04_Status_t HCSR04_DeInit(HCSR04_t *hcsr04,
		TIM_HandleTypeDef *timer_trigger,
		uint32_t Trigger_tim_channel,
		TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_tim_channel_start,
		uint32_t Echo_tim_channel_stop);


#endif /* INC_HCSR04_H_ */
