/*
 * GSR.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Swaroop S Kaimal
 */
#include "stm32l4xx_hal.h"

extern ADC_HandleTypeDef hadc3;
extern uint16_t GSR_VAL;


void Get_GSR(){
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 10);
	GSR_VAL = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
}
