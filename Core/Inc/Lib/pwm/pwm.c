/*
 * pwm.c
 *
 *  Created on: May 23, 2023
 *      Author: santa
 */
#include "pwm.h"
#include "tim.h"

void pwm_set_duty_cicle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cicle)
{
	if (duty_cicle < 0 || duty_cicle > 1)
		return;

	__HAL_TIM_SetCompare(htim, channel, __HAL_TIM_GetAutoreload(htim) * duty_cicle);
}

void pwm_set_period(TIM_HandleTypeDef *htim, float period_ms)
{
	//__HAL_TIM_SetAutoreload(htim, TIM_MS_TO_TICKS(htim, period_ms)); // set the period
	__HAL_TIM_SetAutoreload(htim, period_ms); // non perdiod_ms ma tick
}
