/*
 * pwm.h
 *
 *  Created on: May 23, 2023
 *      Author: santa
 */

#ifndef INC_LIB_PWM_PWM_H_
#define INC_LIB_PWM_PWM_H_

#include "main.h"

void pwm_set_duty_cicle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cicle);
void pwm_set_period(TIM_HandleTypeDef *htim, float period_ms);

#endif /* INC_LIB_PWM_PWM_H_ */
