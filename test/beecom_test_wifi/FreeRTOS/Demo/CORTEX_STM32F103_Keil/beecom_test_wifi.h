#ifndef BEECOM_TEST_WIFI_H
#define BEECOM_TEST_WIFI_H

#include "stm32f10x_usart.h"
#include "stm32f10x_it.h"

#include "beecom_wifi_type.h"
	
#define USART_SEND(usart_type, ch) 		((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)	((ch) = (usart_type)->DR & (uint16_t)0x01FF)

#define LED1TURN() (GPIOA->ODR ^= 1<<8) // red
#define LED2TURN() (GPIOD->ODR ^= 1<<2) // green

#define MAX_SOCK_NUM 			5
#define MAX_SOCK_CLIENT_NUM 	4

#define TASK_BUF_SIZE 	256

#define USART_WIFI 	USART2

enum IPD_STATE {
	IPD_STATE_NONE = 0,
	IPD_STATE_START_PROBE,
	IPD_STATE_HANDLE_HEADER,
	IPD_STATE_HANDLE_SOCKET_ID,
	IPD_STATE_HANDLE_SIZE,
	IPD_STATE_HANDLE_CONTENT,
	IPD_STATE_COMPLETED,
};

void LedInit(void);

void Usart1Init(uint32_t boundrate);
void Usart2Init(uint32_t boundrate);

/* UART interrupt handler. */
volatile void vUARTInterruptHandler( void );

// in stm32f10x_it.h
// void USART2_IRQHandler(void);

// Led Test Task
void vLedTask(void *pvParameters);
void vTcpServerTask(void *pvParameters);

// USART uputs
sint32_t uputs(USART_TypeDef * usart, sint8_t * str);

sint32_t BC_WifiInit(void);

// wifi APIs
// They are similar to linux socket
// just for using the linux net code procedure
// Author: Ansersion
// Date: 2017-03-14
sint32_t BC_Socket(sint32_t family, sint32_t type, sint32_t protocol);
sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen);
sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog);
sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen);
sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen);
sint32_t BC_Close(sint32_t sockfd);
sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags);
sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags);


#endif
