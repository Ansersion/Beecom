#ifndef IRQ_H
#define IRQ_H

#define IrqUsartWifi 		xUSART2_IRQHandler

#define IrqUsartTerminal 	vUARTInterruptHandler

#define USART_SEND(usart_type, ch)      ((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)   ((ch) = (usart_type)->DR & (uint16_t)0x01FF)

#define HFSR 	((uint32_t *)0xE000ED2C)
#define MFSR ((char *)0xE000ED28)
#define BFSR ((char *)0xE000ED29)
#define UFSR ((char *)0xE000ED2A)
#define BFAR ((uint32_t *)0xE000ED38)

volatile void HardFault_Handler(void);
volatile void MemManage_Handler(void);
volatile void BusFault_Handler(void);
volatile void UsageFault_Handler(void);


#endif

