#include "beecom_test_wifi.h"

#include "stm32f10x_nvic.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define BC_CENTER_SERV_PORT 	54321

// Scheme:
// socket 0: server
// socket 1: client
// socket 2: reserved
#define BC_MAX_SOCKET_NUM 		3
#define ASSERT_SOCK_VALID(s) ((s) >= 0 && (s) < BC_MAX_SOCKET_NUM)

// TODO
uint8_t INADDR_ANY[16];

sint8_t xxx;
sint8_t msg[256];

uint8_t SrvBuf[TASK_BUF_SIZE]; 

BC_SocketData sock_data[BC_MAX_SOCKET_NUM];
uint8_t SockBuf_0[1024];
uint8_t SockBuf_1[1024];
uint8_t SockBuf_2[1024];


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
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    /* Enable the USART2 Pins Software Remapping */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    
    /* Configure USART2 Rx (PA.03) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
    
    USART_InitStructure.USART_BaudRate = bound;                
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
    
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
	uint16_t RxData = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == RESET) {
		return;
	}
	USART_RECEIVE(USART2, RxData);
	printf("%c\r\n", (uint8_t)RxData);
}

volatile void vUARTInterruptHandler( void )
{
	/*
	static uint32_t Index = 0;
	uint16_t RxData=0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) {
		return;
	}
	
	if(!IsMsgGotten()) {
		// RxData = USART_ReceiveData(USART1);
		USART1_RECEIVE(RxData);
		RecvBuffer[Index++] = (uint8_t)RxData;// serial_1();
	}
	else {
		// RxData = USART_ReceiveData(USART1); // ignore the data
		RxData = USART1_RECEIVE(RxData);
		return;
	}
	*/
	 
	
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
		while((USART1->SR&0X40)==0)
			;
		USART_SEND(usart, *str++);
	}while('\0' != *str);
	
	return 0;
}

void vLedTask(void *pvParameters)
{
	int xxx_old;
	TickType_t ms = 500;

	xxx = 'a';
	xxx_old = xxx;

	while(1) {
		
		vTaskDelay(ms);
		LED2TURN();
		
		if(xxx != xxx_old) {
			uputs(USART2, "AT+RST");
			xxx_old = xxx;
		}
		sprintf(msg, "Ansersion: %c\r\n", xxx);
		uputs(USART1, msg);
		
	}
}

sint32_t BC_WifiInit(void)
{
	int i;
	TickType_t delay_time_ms = 500;
	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
		memset(&sock_data[i], 0, sizeof(BC_SocketData));
	}
	sock_data[0].buf = SockBuf_0;
	sock_data[1].buf = SockBuf_1;
	sock_data[2].buf = SockBuf_2;
	
	uputs(USART_WIFI, "AT+CWMODE=1");
	vTaskDelay(delay_time_ms);
	
	uputs(USART_WIFI, "AT+RST");
	vTaskDelay(delay_time_ms);

	// SER WIFI SSID ANF PASSWORD
	uputs(USART_WIFI, "AT+CWJAP=\"hb402-hb\",\"68704824\"");
	vTaskDelay(delay_time_ms);
	vTaskDelay(delay_time_ms);
	vTaskDelay(delay_time_ms);
	vTaskDelay(delay_time_ms);
	
	uputs(USART_WIFI, "AT+CIPMUX=1");
	vTaskDelay(delay_time_ms);
	
	sprintf(msg, "AT+CIPSERVER=1,%d", BC_CENTER_SERV_PORT);
	uputs(USART_WIFI, msg);
	vTaskDelay(delay_time_ms);
	
	uputs(USART_WIFI, "AT+CIFSR=?");
	// TODO:
	// Get IP

	return 0;
	
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
	
	return 0;
}

sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	// wait for the semphore
	// copy cliaddr to sock_data
	// set sock_data.valid
	return 0;
}

sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen)
{
	return 0;
}

sint32_t BC_Close(sint32_t sockfd)
{
	return 0;
}

sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	if(!buff) {
		return -3;
	}
	// wait semphore
	// read USART_WIFI(in ISA)
	// taskYIELD
	return 0;
}

sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	if(!buff) {
		return -3;
	}
	
	return 0;
}
