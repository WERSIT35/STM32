#ifndef UART2_H
#define UART2_H

#include "stm32l0xx.h"

void init_USART2(void);
void UART_send(uint8_t data);
void UART_send_string(char *str);


#endif