#ifndef __USART_PRINTF_H
#define __USART_PRINTF_H

#include "main.h"
#include <stdio.h>

void usart_printf_init(UART_HandleTypeDef *huart);
void uart_flush(void);

#endif

