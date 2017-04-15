#ifndef BEECOM_TEST_WIFI_H
#define BEECOM_TEST_WIFI_H

#include "stm32f10x_usart.h"
#include "stm32f10x_it.h"

#include "beecom_wifi_type.h"

extern uint32_t StartReceiveFlag;
	
#define USART_SEND(usart_type, ch) 		((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)	((ch) = (usart_type)->DR & (uint16_t)0x01FF)

#define LED1TURN() (GPIOA->ODR ^= 1<<8) // red
#define LED2TURN() (GPIOD->ODR ^= 1<<2) // green

#define MAX_SOCK_NUM 			5
#define MAX_SOCK_CLIENT_NUM 	4

#define TASK_BUF_SIZE 	256

#define USART_TERMINAL 	USART1
#define USART_WIFI 		USART2


enum {
	WIFI_USART_STATE_INIT = 0,
	WIFI_USART_STATE_RUNNING,
};
#define GET_WIFI_USART_STATE() 	(StartReceiveFlag)
#define SET_WIFI_USART_STATE(F) (StartReceiveFlag=(F))

#define WIFI_MSG_FLAG_GENERAL_OK	0x00000001
#define WIFI_MSG_FLAG_GENERAL_ERR 	0x00000002
#define WIFI_MSG_FLAG_GOT_CONNECT	0x00000004
#define WIFI_MSG_FLAG_GOT_CLOSED	0x00000008

#define WIFI_MSG_FLAG_BUF_OVERFLOW 	0x80000000


#define WIFI_MSG_OK_TERMINATOR_SIZE 		4 // strlen("OK\r\n")
#define WIFI_MSG_ERR_TERMINATOR_SIZE 		7 // strlen("ERROR\r\n")
#define WIFI_MSG_CONNECT_TERMINATOR_SIZE 	9 // strlen("CONNECT\r\n")
#define WIFI_MSG_CLOSED_TERMINATOR_SIZE 	8 // strlen("CLOSED\r\n")

enum IPD_STATE {
	IPD_STATE_NONE = 0,
	IPD_STATE_START_PROBE,
	IPD_STATE_HANDLE_HEADER,
	IPD_STATE_HANDLE_SOCKET_ID,
	IPD_STATE_HANDLE_SIZE,
	IPD_STATE_HANDLE_CONTENT,
	IPD_STATE_COMPLETED,
};

sint32_t BC_Atoi(char n);

void LedInit(void);
void Usart1Init(uint32_t boundrate);
void Usart2Init(uint32_t boundrate);
void BufInit(void);
void SocketInit(void);
sint32_t BC_QueueInit(void);
sint32_t BC_MutexInit(void);
sint32_t BC_Init(void);


volatile void NMI_Handler(void);
volatile void HardFault_Handler(void);
volatile void MemManage_Handler(void);
volatile void BusFault_Handler(void);
volatile void UsageFault_Handler(void);

/* UART interrupt handler. */
volatile void vUARTInterruptHandler( void );

volatile void xUSART2_IRQHandler(void);

// in stm32f10x_it.h
// void USART2_IRQHandler(void);



// Led Test Task
void vLedTask(void *pvParameters);
void vTcpServerTask(void *pvParameters);

// USART uputs
sint32_t uputs(USART_TypeDef * usart, sint8_t * str);

sint32_t BC_WifiInit(void);


sint32_t BC_WifiSetMode(uint32_t mode);

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
