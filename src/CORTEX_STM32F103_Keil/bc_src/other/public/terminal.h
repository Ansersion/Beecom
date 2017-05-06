#ifndef TERMINAL_H
#define TERMINAL_H

#include <irq.h>
#include <bc.h>

#include <beecomint.h>
#include <stm32f10x_usart.h>
#include <stdio.h>

#define USART_TERMINAL 	USART1
#define USART_TERMINAL_BUF_SIZE 	128

void _sys_exit(int x);
int fputc(int ch, FILE *f);

volatile void IrqUsartTerminal(void);
// volatile void vUARTInterruptHandler(void);
void TaskTerminal(void * pvParameters);
sint32_t InitTerm(void);
sint32_t CheckEndFlag(uint8_t * msg, uint32_t msg_size, uint8_t * flag, uint32_t flag_size);


#endif
