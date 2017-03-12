#ifndef BEECOM_TEST_WIFI_H
#define BEECOM_TEST_WIFI_H

#include "stm32f10x_usart.h"
#include "stm32f10x_it.h"

#include "beecomint.h"
	
#define USART_SEND(usart_type, ch) 		((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)	((ch) = (usart_type)->DR & (uint16_t)0x01FF)

#define LED1TURN() (GPIOA->ODR ^= 1<<8) // red
#define LED2TURN() (GPIOD->ODR ^= 1<<2) // green

void LedInit(void);

void Usart1Init(uint32_t boundrate);
void Usart2Init(uint32_t boundrate);

/* UART interrupt handler. */
volatile void vUARTInterruptHandler( void );

// in stm32f10x_it.h
// void USART2_IRQHandler(void);

// Led Test Task
void vLedTask(void *pvParameters);

// USART uputs
sint32_t uputs(USART_TypeDef * usart, sint8_t * str);

#endif
