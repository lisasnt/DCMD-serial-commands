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

#define CMD_N_ARGS_MAX 	3

typedef struct {
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
	GET_PARAM,
	RUN,
	HELP
} motor_cmd_code; //fix name

extern motor_handle motor;// = { .run_time = 4000, .direction = false, .pwm_duty_cycle = .9, .pwm_period = 999};
extern motor_cmd_code motor_code; // = NONE;

void parse_and_execute_cmd(char *cmd_received);
void enable_drv8876(GPIO_PinState state);
void set_motor_direction(GPIO_PinState state);
void run_motor();

#endif /* INC_MOTOR_CMD_UTILS_H_ */
