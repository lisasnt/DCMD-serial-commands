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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_cmd_utils.h"
#include "uart_printf.h"

#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIM 			15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_and_go_to_next_state(state next_state);

void reset_cmd(uart_rx_handler *uart_handler);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
			//HAL_NVIC_SystemReset(); //FIX: what??
			set_and_go_to_next_state(WAIT_FOR_CMD);
			printf("\nDigit 'ok' and press ENTER to start motor test.\r\n");
			printf("Type 'help' to see all the available commands\r\n\n"); //TODO: automatic print of the available cmd
			break;
		case WAIT_FOR_CMD:
			printf("\n\n\rWaiting for command...\r\n\n");
			while(serial_cmd_handler.is_cmd_sent == false)
			{
				HAL_UART_Receive_IT(&UART_USB, (uint8_t *)serial_cmd_handler.last_char_received, 1);
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
			printf("\n\n\rMotor running...\n\r");
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
  * @brief System Clock Configuration
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

/* USER CODE BEGIN 4 */
void set_and_go_to_next_state(state next_state)
{
	fsm_handler.next_state       = next_state;
	fsm_handler.go_to_next_state = true;
}

void reset_cmd(uart_rx_handler *uart_handler)
{
	uart_handler->char_index = 0;
	memset(uart_handler->cmd_received, 0x0, sizeof(uart_handler->cmd_received));
	uart_handler->is_cmd_sent = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	serial_cmd_handler.cmd_received[serial_cmd_handler.char_index] = serial_cmd_handler.last_char_received[0];
	serial_cmd_handler.char_index++;
	if (serial_cmd_handler.last_char_received[0] == '\n' || serial_cmd_handler.last_char_received[0] == '\r')
	{
		serial_cmd_handler.is_cmd_sent = true;
		serial_cmd_handler.char_index = 0;
	}
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
