#ifndef IRQ_H
#define IRQ_H

#define IrqUsartWifi 		xUSART2_IRQHandler

#define IrqUsartTerminal 	vUARTInterruptHandler

#define USART_SEND(usart_type, ch)      ((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)   ((ch) = (usart_type)->DR & (uint16_t)0x01FF)


#endif

