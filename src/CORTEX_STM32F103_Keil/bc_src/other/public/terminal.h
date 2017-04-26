#ifndef TERMINAL_H
#define TERMINAL_H

#include "irq.h"

// #include "stm32f10x_map.h"
#include "beecomint.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#define USART_SEND(usart_type, ch)      ((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)   ((ch) = (usart_type)->DR & (uint16_t)0x01FF)

void _sys_exit(int x);
int fputc(int ch, FILE *f);

volatile void IrqUsartTerminal(void);


#endif
