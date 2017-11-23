//   Copyright 2017 Ansersion
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//

#include <bc_init.h>
#include <panic.h>
#include <bc_type.h>
#include <bc_queue.h>

sint32_t BC_Init(void)
{
	// TickType_t delay_ms = 50;
//	int i = 0;

	// Led indication initialization
	LedInit();
	// USART_TERMINAL initialization
	UsartTermInit(USART_TERMINAL_BAUD_RATE); 
	// USART_WIFI initialization
	UsartWifiInit(USART_WIFI_BAUD_RATE);
	// Buffer initialization
	// BufInit();

	if(BC_QueueInit() != BC_OK) {
		BC_Panic("BC_QueueInit failed");
		return BC_ERR; // never come here
	}

	// SocketInit();

	// if(BC_MutexInit() != BC_OK) {
	// 	sprintf(msg, "MutexInit Error!\r\n");
	// 	uputs(USART_TERMINAL, msg);
	// 	return BC_ERR;
	// }
	return BC_OK;
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

void UsartTermInit(uint32_t bound)
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
	
	/* Usart init£¬9600£¬8bit data bit,1 stop bit, No Parity and flow control, rx tx enable */
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

void UsartWifiInit(uint32_t bound)
{

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

// void BufInit(void)
// {
// 	memset(IPDNum, 0, sizeof(IPDNum));
// 	memset(INADDR_ANY, 0, sizeof(INADDR_ANY));
// 	memset(msg, 0, sizeof(msg));
// 	memset(u16data, 0, sizeof(u16data));
// 	memset(usart_wifi_buf, 0, sizeof(usart_wifi_buf));
// 	memset(SrvBuf, 0, sizeof(SrvBuf));
// }
// 
// void SocketInit(void)
// {
// 	uint32_t i;
// 	
// 	// BC_MAX_SOCKET_NUM == 5;
// 	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
// 		memset(&sock_data[i], 0, sizeof(BC_SocketData));
// 	}
// 	sock_data[0].buf = SockBuf_0;
// 	sock_data[1].buf = SockBuf_1;
// 	sock_data[2].buf = SockBuf_2;
// 	sock_data[3].buf = SockBuf_3;
// 	sock_data[4].buf = SockBuf_4;
// 
// 	sock_data[0].queue_handle = xQueue0;
// 	sock_data[1].queue_handle = xQueue1;
// 	sock_data[2].queue_handle = xQueue2;
// 	sock_data[3].queue_handle = xQueue3;
// 	sock_data[4].queue_handle = xQueue4;
// }
// 
sint32_t BC_QueueInit(void)
{
	uint32_t i = 0;

	for(i = 0; i < BC_ModInQueueSize; i++) {
		BC_ModInQueue[i] = xQueueCreate(BC_CONFIG_QUEUE_ELEMENT_NUMBER, sizeof(BC_QueueElement));
		if(!BC_ModInQueue[i]) {
			return BC_ERR;
		}
	}
	for(i = 0; i < BC_ModOutQueueSize; i++) {
		BC_ModOutQueue[i] = xQueueCreate(BC_CONFIG_QUEUE_ELEMENT_NUMBER, sizeof(BC_QueueElement));
		if(!BC_ModOutQueue[i]) {
			return BC_ERR;
		}
	}

	return BC_OK;
}
// 
// sint32_t BC_MutexInit(void)
// {
// 	sint32_t result = BC_OK;
// 	sint32_t f = 0;
// 	int i;
// 	
// 	if(!(xMutexWifiStateFlag = xSemaphoreCreateBinary())) 	result = BC_ERR;
// 	// if(result != BC_OK) {
// 	// 	while(1) {
// 	// 		for(i=0; i < 10000; i++)
// 	// 			;
// 	// 		sprintf(msg, "MutexInit Error1\r\n");
// 	// 		uputs(USART_TERMINAL, msg);
// 	// 	}
// 
// 	// }
// 	if(!(xMutexRecvFlag = xSemaphoreCreateBinary())) 		result = BC_ERR;
// 	if(result != BC_OK) {
// 		while(1) {
// 			for(i=0; i < 10000; i++)
// 				;
// 			sprintf(msg, "MutexInit Error2\r\n");
// 			uputs(USART_TERMINAL, msg);
// 		}
// 
// 	}
// 	f = SET_START_RECV_FLAG(WIFI_USART_STATE_INIT);
// 	if(f < 0) {
// 		while(1) {
// 			for(i=0; i < 10000; i++)
// 				;
// 			sprintf(msg, "MutexInit Error3:%d\r\n", f);
// 			uputs(USART_TERMINAL, msg);
// 		}
// 	}
// 
// 	return result;
// }
