#ifndef APP_AGENT_TASK_H
#define APP_AGENT_TASK_H

#include "stm32f10x_usart.h"
#include "stm32f10x_it.h"

#include "app_agent_common.h"

#define USART_TERMINAL 	USART1
#define USART_WIFI 		USART2

/* UART interrupt handler. */
volatile void vUARTInterruptHandler( void );

void TaskAppAgent(void *para);

#endif

