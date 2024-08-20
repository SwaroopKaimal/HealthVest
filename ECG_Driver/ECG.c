/*
 * ECG.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Swaroop Kaimal
 */
#include "stm32l4xx_hal.h"

extern ADC_HandleTypeDef hadc2;
extern uint16_t ECG_VAL;

void Get_ECG(){

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	ECG_VAL = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

}



