/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_cmd_utils.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIM 			15
#define CMD_N_ARGS_MAX 	3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint8_t buff[DIM];

//-----------------------------------motor_cmd_utils.h---------------------------------------------------
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

motor_handle motor = { .run_time = 4000, .direction = false, .pwm_duty_cycle = .9, .pwm_period = 999};
//-------------------------------------------------------------------------------------------------------

typedef struct {
	bool is_cmd_sent;
	char cmd_received[DIM];
	char last_char_received[1];
	uint8_t char_index;
} uart_rx_handler;

uart_rx_handler serial_cmd_handler = {.is_cmd_sent = false};

typedef enum {
	INIT = 0U,
	WAIT_FOR_CMD,
	CMD_RECEIVED,
	RUN_MOTOR,
} state;

typedef struct {
	state current_state;
	state next_state;
	bool go_to_next_state;
} state_FSM; // Finite State Machine

state_FSM fsm_handler;

uint8_t motor_code = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void set_and_go_to_next_state(state next_state)
{
	fsm_handler.next_state       = next_state;
	fsm_handler.go_to_next_state = true;
}

void parse_and_execute_cmd(char *cmd_received)
{
	//bool cmd_is_valid = true;
	//uint16_t val;
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
		printf("\nParameters:\n\r");
		printf("run_time:\t %d\tms\r\n", motor.run_time);
		printf("direction:\t %d\n\r", motor.direction);
		uint8_t dc = motor.pwm_duty_cycle*100;
		printf("pwm_duty_cycle:\t %d\t%%\n\r", dc);
		printf("pwm_period:\t %d\ttick\n\r", motor.pwm_period);
	}
	else if (strcmp(arg[0], "help") == 0 || strcmp(arg[0], "help\r") == 0)
	{
		motor_code = HELP;
		printf("\nCommads:\n\r");
		printf("ok\n\r");
		printf("time [ms]\n\r");
		printf("dir [0/1]\n\r");
		printf("pwm [dc%] [n. tick per period]\n\r");
		printf("show\n\r");
	}
	else
	{
		//cmd_is_valid = false;
		printf("\r\nError: invalid command\r\n\n");
	}
}

void reset_cmd(uart_rx_handler *uart_handler)
{
	uart_handler->char_index = 0;
	memset(uart_handler->cmd_received, 0x0, sizeof(uart_handler->cmd_received));
	uart_handler->is_cmd_sent = false;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void enable_drv8876(GPIO_PinState state);
void set_motor_direction(GPIO_PinState state);
void pwm_set_duty_cicle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cicle);
void pwm_set_period(TIM_HandleTypeDef *htim, float period_ms);

void run_motor()
{
	enable_drv8876(SET);
	set_motor_direction(motor.direction);
	pwm_set_period(&htim2, motor.pwm_period);
	pwm_set_duty_cicle(&htim2, TIM_CHANNEL_1, motor.pwm_duty_cycle);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(motor.run_time);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/*void test_motor_debug()
{
	enable_drv8876(SET);
	set_motor_direction(SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	pwm_set_period(&htim2, 999);
	pwm_set_duty_cicle(&htim2, TIM_CHANNEL_1, 0.5);
	HAL_Delay(4000);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	// INIT PHASE
	fsm_handler.current_state    = INIT;
	//fsm_handler.next_state       = WAIT_FOR_CMD;
	//fsm_handler.go_to_next_state = true;
	reset_cmd(&serial_cmd_handler);

	//TODO: vedi righe 186-188

	//test_motor_debug();

	printf("\n\r----------------------START----------------------\r\n");

	/* USER CODE END 2 */


	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (fsm_handler.go_to_next_state) {
			fsm_handler.current_state = fsm_handler.next_state;
		}
		switch (fsm_handler.current_state) {
		case INIT:
			//HAL_NVIC_SystemReset(); //what??
			set_and_go_to_next_state(WAIT_FOR_CMD);
			printf("\nDigit 'ok' and press ENTER to start motor test.\r\n");
			printf("Type 'help' to see available commands\r\n\n"); //TODO: automatic print of the available cmd
			break;
		case WAIT_FOR_CMD:
			printf("\r\nWaiting for command...\r\n\n");
			while(serial_cmd_handler.is_cmd_sent == false)
			{
				HAL_UART_Receive_IT(&huart2, serial_cmd_handler.last_char_received, 1);
			}
			if (serial_cmd_handler.is_cmd_sent)
			{
				set_and_go_to_next_state(CMD_RECEIVED);
			}
			break;
		case CMD_RECEIVED:
			parse_and_execute_cmd(serial_cmd_handler.cmd_received);
			if (motor_code == RUN)
			{
				set_and_go_to_next_state(RUN_MOTOR);
			}
			else
			{
				set_and_go_to_next_state(WAIT_FOR_CMD);
			}
			reset_cmd(&serial_cmd_handler);
			break;
		case RUN_MOTOR:
			printf("\n\rMotor running...\n\r");
			run_motor();
			printf("\n\rEND test\n\r");
			set_and_go_to_next_state(WAIT_FOR_CMD);
			break;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration\
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 499;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, IN2_Pin|nSLEEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : IPROPI_Pin nFAULT_Pin SWO_Pin */
	GPIO_InitStruct.Pin = IPROPI_Pin|nFAULT_Pin|SWO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : IN2_Pin nSLEEP_Pin LED_Pin */
	GPIO_InitStruct.Pin = IN2_Pin|nSLEEP_Pin|LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void enable_drv8876(GPIO_PinState state)
{
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, state);
}

void set_motor_direction(GPIO_PinState state)
{
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, state);
}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//HAL_UART_Transmit(&huart2, serial_cmd_handler.last_char_received[0], 1, 100);
	serial_cmd_handler.cmd_received[serial_cmd_handler.char_index] = serial_cmd_handler.last_char_received[0];
	serial_cmd_handler.char_index++;
	if (serial_cmd_handler.last_char_received[0] == '\n' || serial_cmd_handler.last_char_received[0] == '\r')
	{
		serial_cmd_handler.is_cmd_sent = true;
		serial_cmd_handler.char_index = 0;
	}


	/*cmd_sent = RESET;
	HAL_UART_Transmit(&huart2, rxChr, 1, 100);
	if (rxChr[0] == '\n' || rxChr[0] == '\r'){

		cmd_sent = SET;
		i = 0;
	}
	rxBuffer[i] = rxChr[0];
	i++;*/
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
