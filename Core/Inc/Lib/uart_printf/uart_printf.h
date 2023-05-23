/*
 * uart_printf.h
 *
 *  Created on: May 23, 2023
 *      Author: santa
 */
#include "main.h"

#ifndef INC_LIB_UART_PRINTF_H_
#define INC_LIB_UART_PRINTF_H_

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#endif /* INC_LIB_UART_PRINTF_H_ */
