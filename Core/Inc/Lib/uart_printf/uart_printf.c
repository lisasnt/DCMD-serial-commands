/*
 * uart_printf.c
 *
 *  Created on: May 23, 2023
 *      Author: santa
 */
#include "uart_printf.h"

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&UART_USB, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

