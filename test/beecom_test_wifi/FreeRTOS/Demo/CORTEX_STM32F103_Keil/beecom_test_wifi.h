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

#define WIFI_MSG_FLAG_GOT_CLI 		0x00000010
#define WIFI_MSG_FLAG_GOT_IPD 		0x00000020

#define WIFI_MSG_FLAG_BUF_OVERFLOW 	0x80000000


#define WIFI_MSG_OK_TERMINATOR_SIZE 		4 // strlen("OK\r\n")
#define WIFI_MSG_ERR_TERMINATOR_SIZE 		7 // strlen("ERROR\r\n")
#define WIFI_MSG_CONNECT_TERMINATOR_SIZE 	9 // strlen("CONNECT\r\n")
#define WIFI_MSG_CLOSED_TERMINATOR_SIZE 	8 // strlen("CLOSED\r\n")

enum CIPSTATUS_PARSE_STATE {
	CIPSTATUS_PARSE_NONE = 0,
	CIPSTATUS_PARSE_CHAR_PLUS,
	CIPSTATUS_PARSE_CHAR_COLON,
	CIPSTATUS_PARSE_CID,
	CIPSTATUS_PARSE_TCP_UDP,
	CIPSTATUS_PARSE_CIP_QUOTE_1,
	CIPSTATUS_PARSE_CIP_QUOTE_2,
	CIPSTATUS_PARSE_CIP,
	CIPSTATUS_PARSE_CPORT,
	CIPSTATUS_PARSE_SPORT,
	CIPSTATUS_PARSE_CSTATE,
};

enum IPD_STATE {
	IPD_PARSE_NONE = 0,
	IPD_START_PARSE,
	IPD_PARSE_ID,
	IPD_PARSE_COLON,
	IPD_PARSE_LEN,
	IPD_PARSE_CONTENT,
};

sint32_t BC_Atoi(char n);
sint32_t CheckWifiData(uint8_t * buf, uint32_t buf_size, uint8_t * flag, uint32_t flag_size, bool_t is_end_term);

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

sint32_t GET_START_RECV_FLAG(uint32_t * u32F);
sint32_t SET_START_RECV_FLAG(uint32_t u32F);
sint32_t GET_START_RECV_FLAG_ISR(uint32_t * u32F);
sint32_t SET_START_RECV_FLAG_ISR(uint32_t u32F);

sint32_t ParseCIPSTATUS(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseIPD(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);

#endif
