/*
 * motor_cmd_utils.c
 *
 *  Created on: May 15, 2023
 *      Author: santa
 *
 *  NOTE: enable 'local echo' option in your terminal!
 */
#include "main.h"
#include "motor_cmd_utils.h"
#include "pwm.h"
#include "uart_printf.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

motor_handle motor = { .run_time = 4000, .direction = false, .pwm_duty_cycle = .9, .pwm_period = 1}; //1ms =999tick
motor_cmd_code motor_code = NONE;

void enable_drv8876(GPIO_PinState state)
{
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, state);
}

void set_motor_direction(GPIO_PinState state)
{
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, state);
}

void run_motor()
{
	enable_drv8876(SET);
	set_motor_direction(motor.direction);
	pwm_set_period(&TIM_PWM_DRV8876, motor.pwm_period);
	pwm_set_duty_cicle(&TIM_PWM_DRV8876, TIM_CHANNEL_1, motor.pwm_duty_cycle);
	pwm_start_channel(&TIM_PWM_DRV8876, TIM_CHANNEL_1);
	HAL_Delay(motor.run_time);
	pwm_stop_channel(&TIM_PWM_DRV8876, TIM_CHANNEL_1);
}


void parse_and_execute_cmd(char *cmd_received)
{
	motor_code = NONE;

	/* Get command arguments */
	const char delim[2] = " ";
	//char *token;
	char *arg[CMD_N_ARGS_MAX]; //TODO lunghezza dinamica per possibili comandi futuri
	uint8_t args_index = 0;

	arg[args_index] = strtok(cmd_received, delim); // get the first token (argument)

	while( arg[args_index] != NULL )
	{ // walk through other tokens
		//printf( "%s\n", arg[args_index] );
		args_index++;
		arg[args_index] = strtok(NULL, delim);
	}

	if (strcmp(arg[0], "ok") == 0 || strcmp(arg[0], "ok\r") == 0) //TODO use strstr()
	{
		motor_code = RUN;
	}
	else if (strcmp(arg[0], "time") == 0 && strcmp(arg[1], "") != 0)
	{
		motor_code = SET_TIME;
		motor.run_time = atoi(arg[1]);
	}
	else if (strcmp(arg[0], "dir") == 0 && strcmp(arg[1], "") != 0)
	{
		motor_code = SET_DIR;
		motor.direction = arg[1]; //TODO: code fw and bw as 0 and 1 stringstream
	}
	else if (strcmp(arg[0], "pwm") == 0)
	{
		if (strcmp(arg[1], "") != 0)
		{
			motor_code = SET_PWM_DC;
			float dc = atoi(arg[1])/100.0;
			motor.pwm_duty_cycle = dc;
		}
		if (strcmp(arg[2], "") != 0)
		{
			motor_code = SET_PWM_PERIOD;
			motor.pwm_period = atoi(arg[2]);
		}
	}
	else if (strcmp(arg[0], "show") == 0 || strcmp(arg[0], "show\r") == 0)
	{
		motor_code = GET_PARAM;
		printf("\n\nCurrent parameters:\n\r");
		printf("run_time:\t %d\tms\r\n", motor.run_time);
		printf("direction:\t %d\t0=fw 1=bw\n\r", motor.direction);
		uint8_t dc = motor.pwm_duty_cycle*100;
		printf("pwm_duty_cycle:\t %d\t%%\n\r", dc);
		printf("pwm_period:\t %d\tms\n\r", motor.pwm_period);
	}
	else if (strcmp(arg[0], "help") == 0 || strcmp(arg[0], "help\r") == 0)
	{
		motor_code = HELP;
		printf("\n\nCommads:\n\r");
		printf("\nok\n\r");
		printf("\n\tRun the motor with the last modified settings.\r\n\tIf there isn't any change it keeps the default settings.\r\n");
		printf("\ntime [ms]\n\r");
		printf("\n\tSet the time of the running test in ms.\r\n");
		printf("\ndir [0/1]\n\r");
		printf("\n\tSet the motor spin direction.\r\n\t0 for forward or 1 for backward.\r\n");
		printf("\npwm [dc%%] [period_ms]\n\r");
		printf("\n\tSet the parameter of the pwm signal.\r\n\t[dc%%] : duty-cycle in %%.\r\n\t[period_ms] : time period of the wave in ms.\r\n\tNote that [period_ms] is negligible, e.g. 'pwm 50'+ENTER works!\r\n");
		printf("\nshow\n\r");
		printf("\n\tDisplay the current motor test settings.\r\n");
		printf("\nhelp\n\r");
		printf("\n\tDisplay the commands manual.\r\n");
	}
	else
	{
		printf("\r\nError: invalid command\r\n\n");
	}
}
