#include "beecom_test_wifi.h"

#include "stm32f10x_nvic.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define BC_CENTER_SERV_PORT 	54321

#define HFSR 	((uint32_t *)0xE000ED2C)
#define MFSR ((char *)0xE000ED28)
#define BFSR ((char *)0xE000ED29)
#define UFSR ((char *)0xE000ED2A)
#define BFAR ((uint32_t *)0xE000ED38)

// Scheme:
// socket 0: server
// socket 1: client
// socket 2: reserved
#define BC_MAX_SOCKET_NUM 		5
#define ASSERT_SOCK_VALID(s) ((s) >= 0 && (s) < BC_MAX_SOCKET_NUM)

#define NUM_ASCII_SIZE 	6
uint8_t IPDNum[NUM_ASCII_SIZE];

// TODO
uint8_t INADDR_ANY[16];
uint32_t StartReceiveFlag = 0;
uint32_t StartAcceptFlag = 0;
BC_Sockaddr * k_cliaddr = NULL;
uint32_t * k_addrlen = NULL;

sint8_t xxx;
sint8_t msg[256];
uint8_t u16data[32];

#define WIFI_BUF_SIZE 	4096
uint8_t usart_wifi_buf[WIFI_BUF_SIZE];

uint8_t SrvBuf[TASK_BUF_SIZE]; 

// TODO:
// "WifiRecvFlag" need a mutex
uint32_t WifiRecvFlag = 0; // general recv flag
BC_SocketData sock_data[BC_MAX_SOCKET_NUM];

#define SOCKET_BUF_SIZE 1024
// BC_MAX_SOCKET_NUM sock buffer
uint8_t SockBuf_0[SOCKET_BUF_SIZE];
uint8_t SockBuf_1[SOCKET_BUF_SIZE];
uint8_t SockBuf_2[SOCKET_BUF_SIZE];
uint8_t SockBuf_3[SOCKET_BUF_SIZE];
uint8_t SockBuf_4[SOCKET_BUF_SIZE];


static QueueHandle_t xQueue0 = NULL;
static QueueHandle_t xQueue1 = NULL;
static QueueHandle_t xQueue2 = NULL;
static QueueHandle_t xQueue3 = NULL;
static QueueHandle_t xQueue4 = NULL;

sint32_t BC_Atoi(char n)
{
	return n - '0';
}


void LedInit(void)
{		
	GPIO_InitTypeDef GPIO_InitType;
	GPIO_TypeDef * GPIO_Type;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD, ENABLE);
	
	// GPIO output initialization;
	GPIO_InitType.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitType.GPIO_Speed = GPIO_Speed_50MHz;
	
	// Initialize PA.8
	GPIO_Type = GPIOA;
	GPIO_InitType.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIO_Type, &GPIO_InitType);
	
	// Initialize PD.2
	GPIO_Type = GPIOD;
	GPIO_InitType.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIO_Type, &GPIO_InitType);
}

void Usart1Init(uint32_t bound)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	/* USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	USART_DeInit(USART1);
	
	/* Usart init，9600，8bit data bit,1 stop bit, No Parity and flow control, rx tx enable */
	USART_InitStructure.USART_BaudRate               = bound;
	USART_InitStructure.USART_WordLength             = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits               = USART_StopBits_1;
	USART_InitStructure.USART_Parity                 = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl    = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                   = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		
	USART_Cmd(USART1, ENABLE);
}

void Usart2Init(uint32_t bound)
{
	/*
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// Enable GPIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	// USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	USART_DeInit(USART2);
	
	// Usart init，9600，8bit data bit,1 stop bit, No Parity and flow control, rx tx enable
	USART_InitStructure.USART_BaudRate               = bound;
	USART_InitStructure.USART_WordLength             = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits               = USART_StopBits_1;
	USART_InitStructure.USART_Parity                 = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl    = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                   = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		
	USART_Cmd(USART2, ENABLE);
	*/
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // Enable the USART2 Pins Software Remapping
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    
    // Configure USART2 Rx (PA.03) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure USART2 Tx (PA.02) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
    // Enable the USART2 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
	
	USART_DeInit(USART2);
    
    USART_InitStructure.USART_BaudRate = bound;                
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
    
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // Enable USART2
    USART_Cmd(USART2, ENABLE);
	
}

volatile void xUSART2_IRQHandler(void)
{	
	static uint32_t Index = 0;
	static uint16_t u16PackSize = 0;
	static uint32_t IPDState = IPD_STATE_START_PROBE;
	// static uint32_t 
	static uint16_t num = 0;
	static uint8_t i = 0;
	static uint16_t FirstComma = 0, SecondComma = 0, Colon = 0;
	static uint16_t RxData=0;
	static uint16_t u16SocketID = 0;
	static sint32_t u32CurrentSockId = -1;
	uint32_t u32Tmp = 0;
	
	// uputs(USART_TERMINAL, "u2\r\n");
	if(USART_GetITStatus(USART_WIFI, USART_IT_RXNE) == RESET) {
		return;
	}
	RxData = USART_RECEIVE(USART_WIFI, RxData);
	// i = (char)(0xFF & RxData);
	// sprintf(u16data, "%02x ", i);
	// uputs(USART_TERMINAL, u16data);
	// i = 0;
	// return;
	
	if(GET_WIFI_USART_STATE() == WIFI_USART_STATE_RUNNING) {
		usart_wifi_buf[Index++] = (uint8_t)RxData;
		if(Index >= WIFI_BUF_SIZE - 1) {
			// StartReceiveFlag = 0;
			// SET_WIFI_USART_STATE(WIFI_MSG_FLAG_BUF_OVERFLOW);
			IPDState = IPD_STATE_START_PROBE;
			usart_wifi_buf[WIFI_BUF_SIZE-1] = '\0';
			Index = 0;
			return;
		}
		// IPD PACKET
		switch(IPDState) {
			case IPD_STATE_START_PROBE:
				if('+' == usart_wifi_buf[0]) {
					IPDState = IPD_STATE_HANDLE_HEADER;
				}
				break;
				// Not return, 
				// because we don't know whether it's an another packet type
			case IPD_STATE_HANDLE_HEADER:
				if( Index >= 4) {
					if (usart_wifi_buf[0] == '+' && 
						usart_wifi_buf[1] == 'I' && 
						usart_wifi_buf[2] == 'P' && 
						usart_wifi_buf[3] == 'D' 	) {
							IPDState = IPD_STATE_HANDLE_SOCKET_ID;
						} else {
							IPDState = IPD_STATE_START_PROBE;
						}
				}
				// Not return, 
				// because we don't know whether it's an another packet type
				break;
			case IPD_STATE_HANDLE_SOCKET_ID:
				if(Index <= 4) {
					// retrive the previous state
					IPDState = IPD_STATE_START_PROBE;
					break;
				}
				FirstComma = 0;
				SecondComma = 0;
				for(i = 4; i < Index; i++) {
					if(',' == usart_wifi_buf[i] && 0 == FirstComma) {
						FirstComma = i;
					}
					else if(',' == usart_wifi_buf[i] && FirstComma != 0) {
						SecondComma = i;
						break;
					}
				}
				if(0 == FirstComma || 0 == SecondComma) {
					return;
				}
				u32Tmp = SecondComma-FirstComma-1; // number of digit
				if(u32Tmp> NUM_ASCII_SIZE - 1) {  // "-1" for '\0'
					IPDState = IPD_STATE_START_PROBE;
					break;
				}
				memcpy((void *)IPDNum, (void *)usart_wifi_buf[FirstComma+1], u32Tmp);
				IPDNum[u32Tmp] = '\0';
				u32Tmp = atoi(IPDNum);
				if(u32Tmp < 0 || u32Tmp > 4) {
					// out of range
					IPDState = IPD_STATE_START_PROBE;
					break;
				}
				// u16SocketID = u32Tmp;
				u32CurrentSockId = u32Tmp;
				IPDState = IPD_STATE_HANDLE_SIZE;
				return;
				break; // never come here
			case IPD_STATE_HANDLE_SIZE:
				Colon = 0;
				SecondComma = 0;
				for(i = Index - 1; i > 4; i--) {
					if(';' == usart_wifi_buf[i] && 0 == Colon) {
						Colon = i;
					}
					else if(',' == usart_wifi_buf[i] && Colon != 0) {
						SecondComma = i;
						break;
					}
				}
				if(0 == Colon || 0 == SecondComma) {
					return;
				}
				u32Tmp = Colon - SecondComma - 1; // number of digit
				if(u32Tmp> NUM_ASCII_SIZE - 1) {  // "-1" for '\0'
					IPDState = IPD_STATE_START_PROBE;
					break;
				}
				memcpy((void *)IPDNum, (void *)&usart_wifi_buf[SecondComma+1], u32Tmp);
				IPDNum[u32Tmp] = '\0';
				u32Tmp = atoi(IPDNum);
				if(u32Tmp > 5000) {  // max msg size 5000(temporarily 5000)
					// out of range
					IPDState = IPD_STATE_START_PROBE;
					break;
				}
				u16PackSize = u32Tmp;
				IPDState = IPD_STATE_HANDLE_CONTENT;
				return;
				break; // never come here
			case IPD_STATE_HANDLE_CONTENT:
				if(Index > Colon + u16PackSize) {
					IPDState = IPD_STATE_COMPLETED;
				} else {
					return;
					break; // never come here
				}
				// no break here
			case IPD_STATE_COMPLETED:
				// TODO:
				// handle \r\n left;
				if(u16PackSize > SOCKET_BUF_SIZE) {
					IPDState = IPD_STATE_START_PROBE;
				} else if(u16SocketID > 4) {
					IPDState = IPD_STATE_START_PROBE;
				} else {
					if(usart_wifi_buf[Index-1] == '\n' && usart_wifi_buf[Index-2] == '\r') {
						memcpy(sock_data[u32CurrentSockId].buf, &usart_wifi_buf[Colon+1], u16PackSize);
						IPDState = IPD_STATE_START_PROBE;
					}
				}
				
				return;
			default:
				IPDState = IPD_STATE_START_PROBE;
				break;
		}
		
		// client connects to local server
		if(Index >= 1) {
			if(isdigit(usart_wifi_buf[0])) {
				u32CurrentSockId = BC_Atoi(usart_wifi_buf[0]);
				if(Index >= WIFI_MSG_CONNECT_TERMINATOR_SIZE) {
					if( usart_wifi_buf[Index-1] == '\n'	&& 
						usart_wifi_buf[Index-2] == '\r'	&&
						usart_wifi_buf[Index-3] == 'T' 	&&
						usart_wifi_buf[Index-4] == 'C' 	&&
						usart_wifi_buf[Index-5] == 'E' 	&&	
						usart_wifi_buf[Index-6] == 'N' 	&&
						usart_wifi_buf[Index-7] == 'N' 	&&
						usart_wifi_buf[Index-8] == 'O' 	&&
						usart_wifi_buf[Index-9] == 'C' 	) {
							sock_data[u32CurrentSockId].msg_flag |= WIFI_MSG_FLAG_GOT_CONNECT;
							return;
						}							
				}
				if(Index >= WIFI_MSG_CLOSED_TERMINATOR_SIZE) {
					if( usart_wifi_buf[Index-1] == '\n'	&& 
						usart_wifi_buf[Index-2] == '\r'	&&
						usart_wifi_buf[Index-3] == 'D' 	&&
						usart_wifi_buf[Index-4] == 'E' 	&&	
						usart_wifi_buf[Index-5] == 'S' 	&&
						usart_wifi_buf[Index-6] == 'O' 	&&
						usart_wifi_buf[Index-7] == 'L' 	&&
						usart_wifi_buf[Index-8] == 'C' 	) {
							sock_data[u32CurrentSockId].msg_flag |= WIFI_MSG_FLAG_GOT_CLOSED;
							return;
						}							
				}
			}
		}
		if(Index >= 10) {
			if( usart_wifi_buf[Index-1] == 'S'	&& 
				usart_wifi_buf[Index-2] == 'U'	&&
				usart_wifi_buf[Index-3] == 'T' 	&&
				usart_wifi_buf[Index-4] == 'A' 	&&
				usart_wifi_buf[Index-5] == 'T' 	&&	
				usart_wifi_buf[Index-6] == 'S' 	&&
				usart_wifi_buf[Index-7] == 'P' 	&&
				usart_wifi_buf[Index-8] == 'I' 	&&
				usart_wifi_buf[Index-9] == 'C' 	&& 
				usart_wifi_buf[Index-10] =='+' 	) {
				}	
		}
		// END TERMINAL PACKET
		if(Index >= WIFI_MSG_OK_TERMINATOR_SIZE) {
			if( usart_wifi_buf[Index-1] == '\n'	&& 
				usart_wifi_buf[Index-2] == '\r'	&&
				usart_wifi_buf[Index-3] == 'K' 	&&
				usart_wifi_buf[Index-4] == 'O' 	) 
			{
					usart_wifi_buf[Index] = '\0';
					WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
					Index = 0;
					return;
			}
		}
		if(Index >= WIFI_MSG_ERR_TERMINATOR_SIZE) {
			if( usart_wifi_buf[Index-1] == '\n'	&& 
				usart_wifi_buf[Index-2] == '\r'	&&
				usart_wifi_buf[Index-3] == 'R' 	&&
				usart_wifi_buf[Index-4] == 'O' 	&&
				usart_wifi_buf[Index-5] == 'R' 	&&
				usart_wifi_buf[Index-6] == 'R' 	&&
				usart_wifi_buf[Index-7] == 'E' 	) 
			{
					usart_wifi_buf[Index] = '\0';
					WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_ERR;
					Index = 0;
					return;
			}
		}
	} else if(GET_WIFI_USART_STATE() == WIFI_USART_STATE_INIT) {
		usart_wifi_buf[Index++] = (uint8_t)RxData;
		if(Index >= WIFI_BUF_SIZE - 1) {
			// StartReceiveFlag = 0;
			WifiRecvFlag |= WIFI_MSG_FLAG_BUF_OVERFLOW;
			IPDState = IPD_STATE_START_PROBE;
			usart_wifi_buf[WIFI_BUF_SIZE-1] = '\0';
			Index = 0;
			return;
		}
		// WifiRecvFlag = 0;
		// END TERMINAL PACKET
		if(Index >= WIFI_MSG_OK_TERMINATOR_SIZE) {
			if( usart_wifi_buf[Index-1] == '\n'	&& 
				usart_wifi_buf[Index-2] == '\r'	&&
				usart_wifi_buf[Index-3] == 'K' 	&&
				usart_wifi_buf[Index-4] == 'O' 	) 
			{
					usart_wifi_buf[Index] = '\0';
					WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
					Index = 0;
					return;
			}
		}
		if(Index >= WIFI_MSG_ERR_TERMINATOR_SIZE) {
			if( usart_wifi_buf[Index-1] == '\n'	&& 
				usart_wifi_buf[Index-2] == '\r'	&&
				usart_wifi_buf[Index-3] == 'R' 	&&
				usart_wifi_buf[Index-4] == 'O' 	&&
				usart_wifi_buf[Index-5] == 'R' 	&&
				usart_wifi_buf[Index-6] == 'R' 	&&
				usart_wifi_buf[Index-7] == 'E' 	) 
			{
					usart_wifi_buf[Index] = '\0';
					WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_ERR;
					Index = 0;
					return;
			}
		}
		
	}
	return;
}

volatile void HardFault_Handler(void)
{
	sprintf(msg, "HardFault:%x\r\n", *HFSR);
	sprintf(msg+strlen(msg), "MemManage:%x\r\n", *MFSR);
	sprintf(msg+strlen(msg), "BusFault:%x\r\n", *BFSR);
	sprintf(msg+strlen(msg), "BFAR:%x\r\n", *BFAR);
	sprintf(msg+strlen(msg), "UsageFault:%x\r\n", *UFSR);
	uputs(USART1, msg);
	for(;;);
}

volatile void MemManage_Handler(void)
{

	sprintf(msg, "MemManage:%x\r\n", *MFSR);
	uputs(USART1, msg);
	for(;;);
}

volatile void BusFault_Handler(void)
{

	sprintf(msg, "BusFault:%x\r\n", *BFSR);
	sprintf(msg+strlen(msg), "BFAR:%x\r\n", *BFAR);
	uputs(USART1, msg);
	for(;;);
}

volatile void UsageFault_Handler(void)
{

	sprintf(msg, "UsageFault:%x\r\n", *UFSR);
	uputs(USART1, msg);
	for(;;);
}

volatile void vUARTInterruptHandler( void )
{
	uint16_t RxData = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) {
		return;
	}
	USART_RECEIVE(USART1, RxData);
	xxx = (sint8_t)RxData;
}

sint32_t uputs(USART_TypeDef * usart, sint8_t * str)
{
	if(!usart || !str) {
		return -1;
	}
	if('\0' == *str) {
		return 0;
	}
	do {
		// 检测发送完成TC标志。
		// 为了提高函数效率，此处没有使用库函数USART_GetITStatus
		// while((usart->SR&0X40)==0)
		//	;
		// USART_SEND(usart, *str++);
		USART_SendData(usart,*str++);
		//delay_ms(1);
		while(USART_GetFlagStatus(usart,USART_FLAG_TC)!=SET);//等待发送结束
	}while('\0' != *str);
	
	return 0;
}

void vLedTask(void *pvParameters)
{
	int xxx_old;
	TickType_t ms = 500;
	int init;
	

	xxx = 'a';
	xxx_old = xxx;

	while(1) {
		
		vTaskDelay(ms);
		LED2TURN();
		
		if(xxx != xxx_old) {
			xxx_old = xxx;
		}
		if(xxx == 'w') {
			if(init == 0) {
				init = 1;
				uputs(USART1, "Start WifiInit\r\n");
				BC_WifiInit();
			}
			uputs(USART1, INADDR_ANY);
			uputs(USART1, "\r\n");
		} else {
			init = 0;
			sprintf(msg, "Ansersion: %c\r\n", xxx);
			uputs(USART1, msg);
			// sprintf(msg, "Ansersion2: %c\r\n", xxx);
			// uputs(USART2, msg);
		}
		
	}
}

sint32_t BC_WifiInit(void)
{
	int i = 0;
	char * str_pos = NULL;
	TickType_t delay_time_ms = 2000;
	uint8_t * tmp_ptr = NULL;
	
	// StartAcceptFlag = 0;
	// StartReceiveFlag = 0;
	
	if(BC_CreateQueue() != BC_OK) {
		uputs(USART_TERMINAL, "BC_CreateQueue: Error\r\n");
		return -1;
	}
	
	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
		memset(&sock_data[i], 0, sizeof(BC_SocketData));
	}
	sock_data[0].buf = SockBuf_0;
	sock_data[1].buf = SockBuf_1;
	sock_data[2].buf = SockBuf_2;
	sock_data[3].buf = SockBuf_3;
	sock_data[4].buf = SockBuf_4;

	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		uputs(USART_WIFI, "AT+CWMODE=1\r\n");
		vTaskDelay(delay_time_ms);
		uputs(USART_TERMINAL, "\r\n");
		uputs(USART_TERMINAL, usart_wifi_buf);
		uputs(USART_TERMINAL, "\r\n");
		
		uputs(USART_TERMINAL, "AT+CWMODE=1...\r\n");
	}
	uputs(USART_TERMINAL, "AT+CWMODE=1: OK\r\n");
	memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;
	
	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		uputs(USART_WIFI, "AT+RST\r\n");
		vTaskDelay(delay_time_ms);
		sprintf(msg, "WifiRecvFlag: %02x\r\n", WifiRecvFlag);
		uputs(USART_TERMINAL, msg);
		uputs(USART_TERMINAL, usart_wifi_buf);
		usart_wifi_buf[0] = '\0';
		uputs(USART_TERMINAL, "AT+RST...\r\n");
	}
	uputs(USART_TERMINAL, "AT+RST: OK\r\n");
	memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;

	// SER WIFI SSID AND PASSWORD
	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		uputs(USART_WIFI, "AT+CWJAP=\"hb402-2g\",\"68704824\"\r\n");
		vTaskDelay(delay_time_ms);
		sprintf(msg, "WifiRecvFlag: %02x\r\n", WifiRecvFlag);
		uputs(USART_TERMINAL, msg);
		uputs(USART_TERMINAL, usart_wifi_buf);
		usart_wifi_buf[0] = '\0';
		uputs(USART_TERMINAL, "AT+CWJAP=\"hb402-2g\",\"68704824\"...\r\n");
	}
	uputs(USART_TERMINAL, "AT+CWJAP=\"hb402-2g\",\"68704824\": OK\r\n");
	memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;
	
	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		uputs(USART_WIFI, "AT+CIPMUX=1\r\n");
		vTaskDelay(delay_time_ms);
		uputs(USART_TERMINAL, "AT+CIPMUX=1...\r\n");
	}
	uputs(USART_TERMINAL, "AT+CIPMUX=1: OK\r\n");
	memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;
	
	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		sprintf(msg, "AT+CIPSERVER=1,%d\r\n", BC_CENTER_SERV_PORT);
		uputs(USART_WIFI, msg);
		vTaskDelay(delay_time_ms);
		sprintf(msg, "AT+CIPSERVER=1,%d...\r\n", BC_CENTER_SERV_PORT);
		uputs(USART_TERMINAL, msg);
	}
	sprintf(msg, "AT+CIPSERVER=1,%d: OK\r\n", BC_CENTER_SERV_PORT);
	uputs(USART_TERMINAL, msg);
	memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;
	
	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		uputs(USART_WIFI, "AT+CIFSR\r\n");
		vTaskDelay(delay_time_ms);
		uputs(USART_TERMINAL, "AT+CIFSR\r\n");
	}
	uputs(USART_TERMINAL, "AT+CIFSR: OK\r\n");
	// memset(usart_wifi_buf, 0, WIFI_BUF_SIZE);
	WifiRecvFlag = 0;

	tmp_ptr = usart_wifi_buf;
	tmp_ptr = strstr(tmp_ptr, "STAIP,");
	if(!tmp_ptr) {
		uputs(USART_TERMINAL, "Parse IP: Error\r\n");
		return -2;
	}
	tmp_ptr += strlen("STAIP,");
	// tmp_ptr += 1; // at position of "
	tmp_ptr += 1; // at the first number 
	uputs(USART_TERMINAL, tmp_ptr);
	uputs(USART_TERMINAL, "\r\n");
	memset(INADDR_ANY, 0, 16);
	for(i = 0; i < 16;i++) {
		if('.' == tmp_ptr[i] || isdigit(tmp_ptr[i])) {
			INADDR_ANY[i] = tmp_ptr[i]; 
		} else {
			break;
		}
	}
	INADDR_ANY[i] = '\0';
	sprintf(msg, "My IP is %s\r\n", INADDR_ANY);
	uputs(USART_TERMINAL, msg);
	
	return 0;
}

sint32_t BC_CreateQueue(void)
{
	sint32_t result = BC_OK;
	
	if(!(xQueue0 = xQueueCreate(1, sizeof(BC_SocketData)))) result |= BC_ERR;
	if(!(xQueue1 = xQueueCreate(1, sizeof(BC_SocketData)))) result |= BC_ERR;
	if(!(xQueue2 = xQueueCreate(1, sizeof(BC_SocketData)))) result |= BC_ERR;
	if(!(xQueue3 = xQueueCreate(1, sizeof(BC_SocketData)))) result |= BC_ERR;
	if(!(xQueue4 = xQueueCreate(1, sizeof(BC_SocketData)))) result |= BC_ERR;
	
	return result;
}

void vTcpServerTask(void *pvParameters)
{
	BC_Sockaddr server_addr;
	BC_Sockaddr client_addr;
	sint32_t server_socket = 0;
	sint32_t client_socket;
	uint32_t addr_len = sizeof(BC_Sockaddr);
	sint32_t trans_len = 0;
	
	memset(&server_addr, 0, sizeof(BC_Sockaddr));
	server_addr.sin_family = AF_INET;
	// diff: string not long
	memcpy(server_addr.sin_addr.s_addr, INADDR_ANY, sizeof(INADDR_ANY));
	// diff: host order not net order
	server_addr.sin_port = BC_CENTER_SERV_PORT;
	
	server_socket = BC_Socket(AF_INET, SOCK_STREAM, 0);
	if(server_socket < 0) {
		printf("Error: Create socket failed\r\n");
		vTaskDelete(NULL);
		return;
	}
	if(BC_Bind(server_socket,&server_addr,sizeof(server_addr)) != 0) {
        printf("Server Bind Port : %d Failed!\r\n", server_addr.sin_port); 
        vTaskDelete(NULL);
		return;
    }
	if (BC_Listen(server_socket, LISTEN_QUEUE)) {
        printf("Server Listen Failed!\r\n"); 
        vTaskDelete(NULL);
		return;
    }
	
	while(TRUE) {
		memset(&client_addr, 0, sizeof(BC_Sockaddr));
		memset(SrvBuf, 0, TASK_BUF_SIZE);
		client_socket = BC_Accept(server_socket, &client_addr,&addr_len);
        if (client_socket < 0) {
            printf("Server Accept Failed!\r\n");
            break;
        }
		trans_len = BC_Recv(client_socket, SrvBuf, TASK_BUF_SIZE, 0);
		if(trans_len <= 0) {
			printf("Server Recv Failed\r\n");
			BC_Close(client_socket);
			continue;
		}
		printf("Recv from %s:%d\r\n", client_addr.sin_addr.s_addr, client_addr.sin_port);
		printf("Msg: #%s#\r\n", SrvBuf);
		printf("Echoing now...\r\n");
		trans_len = BC_Send(client_socket, SrvBuf, TASK_BUF_SIZE, 0);
		if(trans_len < 0) {
			printf("Server Send Failed\r\n");
			BC_Close(client_socket);
			continue;
		}
		BC_Close(client_socket);
	}
	
	BC_Close(server_socket);
	vTaskDelete(NULL);
	return;
}

sint32_t BC_Socket(sint32_t family, sint32_t type, sint32_t protocol)
{
	int i;
	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
		if(!sock_data[i].valid) {
			sock_data[i].valid = TRUE;
			return i;
		}
	}
	return -1;
}

sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!myaddr) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}
	memcpy(&(sock_data[sockfd].addr), myaddr, addrlen);
	return 0;
}

sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(backlog < 0) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}
	sock_data[sockfd].backlog = backlog;
	sprintf(msg, "AT+CIPSERVER=1,%d", sock_data[sockfd].addr.sin_port);
	uputs(USART_WIFI, msg);
	sprintf(msg, "Server info: IP=%s, PORT=%d\r\n", INADDR_ANY, sock_data[sockfd].addr.sin_port);
	uputs(USART1, msg);
	
	return 0;
}

sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!cliaddr) {
		return -2;
	}
	if(!addrlen) {
		return -3;
	}
	if(!sock_data[sockfd].valid) {
		return -4;
	}
	// wait for the semphore
	
	StartReceiveFlag = 1;
	StartAcceptFlag = 1;
	
	k_cliaddr = cliaddr;
	k_addrlen = addrlen;
	while(StartAcceptFlag)
		;
	// TODO:
	// check the server sockfd
	
	return 0;
}

sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!servaddr) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}

	sprintf(msg, "AT+CIPSTART=%d,\"TCP\",\"%s\",%d", sockfd, servaddr->sin_addr.s_addr, servaddr->sin_port);
	return 0;
}

sint32_t BC_Close(sint32_t sockfd)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	sprintf(msg, "AT+CIPCLOSE=%d", sockfd);
	uputs(USART_WIFI, msg);
	sock_data[sockfd].valid = FALSE;
	return 0;
}

sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}
	// wait semphore
	StartReceiveFlag = 1;
	while(StartReceiveFlag)
		;
	// read USART_WIFI(in ISA)
	// taskYIELD
	return 0;
}

sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}

	// memset(msg, 0, sizeof(msg));
	
	sprintf(msg, "AT+CIPSEND=0,%d", nbytes);
	uputs(USART_WIFI, msg);
	vTaskDelay(10); // TODO
	uputs(USART_WIFI, (sint8_t *)buff);
	vTaskDelay(10); // TODO
	
	return 0;
}
