/*
 * motor_cmd_utils.h
 *
 *  Created on: May 15, 2023
 *      Author: santa
 */

#ifndef INC_MOTOR_CMD_UTILS_H_
#define INC_MOTOR_CMD_UTILS_H_

#include "main.h"
#include <stdbool.h>

//#define FREQUENCY_25_KHZ_IN_MS 0.04
#define MAX_ALLOWED_PWM_DUTY_CYCLE   1
#define MIN_ALLOWED_PWM_DUTY_CYCLE   0

/*typedef struct {
	int run_time;
	bool direction;
	float pwm_duty_cycle;
	int pwm_period; // TODO: no period in tick ma in ms
} motor_handle;

typedef enum {
	NONE = 0U,
	SET_TIME,
	SET_DIR,
	SET_PWM_DC,
	SET_PWM_PERIOD,
	START
} motor_cmd_code;*/

/*typedef struct {
    motor_cmd_code cmd;
    uint32_t val;
} motor_cmd_msg;*/

//extern motor_handle motor; // = { .run_time = 4000, .direction = false, .pwm_duty_cycle = .9, .pwm_period = 999};

//TODO: funzioni
//void motor_init
//void setMotor_run_time(motor_handle *motor, int value);

#endif /* INC_MOTOR_CMD_UTILS_H_ */
